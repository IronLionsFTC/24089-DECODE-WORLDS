package org.firstinspires.ftc.teamcode.systems;

import org.firstinspires.ftc.teamcode.lioncore.hardware.LionCRServo;
import org.firstinspires.ftc.teamcode.lioncore.hardware.LionMotor;
import org.firstinspires.ftc.teamcode.lioncore.math.types.Vector2;
import org.firstinspires.ftc.teamcode.lioncore.tasks.TaskOpMode;
import org.firstinspires.ftc.teamcode.parameters.Zeroing;

public class SwervePod {

    private final LionMotor   motor;
    private final LionCRServo servo;
    private final Vector2     offset;

    public double currentAngle = 0;
    public double targetAngle  = 0;

    private double lastPodAngle      = 0;
    private long   lastTime          = System.nanoTime();
    private double angularVelocity   = 0;

    private double lastAngleSetpoint = 0;
    private long   lastSetpointTime  = System.nanoTime();

    public SwervePod(LionMotor motor, LionCRServo servo,
                     Vector2 offset, double startDegrees) {
        this.motor  = motor;
        this.servo  = servo;
        this.offset = offset;
        motor.resetPositionTo(startDegrees * (4096.0 / 360.0));
    }

    public double update(Vector2 translation, double omega) {

        double podAngle = readAngle();
        trackVelocity(podAngle);

        double predictedAngle = podAngle + angularVelocity * SwerveDrive.PodControl.latency;
        currentAngle = podAngle;

        Vector2 rotComponent  = Vector2.cartesian(omega * offset.y(), -omega * offset.x());
        Vector2 finalVelocity = translation.add(rotComponent);

        if (finalVelocity.magnitude() < 1e-6) {
            servo.setPower(0);
            return 0;
        }

        double rawDesired = -finalVelocity.polarDirection();
        double fwdAngle   = closestEquivalentAngle(rawDesired,       podAngle);
        double revAngle   = closestEquivalentAngle(rawDesired + 180, podAngle);

        double angleSetpoint, drivePower;
        if (Math.abs(wrapDeg(fwdAngle - podAngle)) <= Math.abs(wrapDeg(revAngle - podAngle))) {
            angleSetpoint = fwdAngle;
            drivePower    =  finalVelocity.magnitude();
        } else {
            angleSetpoint = revAngle;
            drivePower    = -finalVelocity.magnitude();
        }

        targetAngle = angleSetpoint;
        commandServo(angleSetpoint, predictedAngle, podAngle);

        double alignError   = Math.abs(wrapDeg(angleSetpoint - podAngle));
        double scaleFactor  = Math.max(0, Math.cos(Math.toRadians(alignError)));
        return drivePower * scaleFactor;
    }

    public double updateIdle(boolean oPattern) {

        double podAngle = readAngle();
        trackVelocity(podAngle);
        currentAngle = podAngle;

        if (oPattern) {
            double tangent = wrapDeg(offset.polarDirection() + 90);
            double optA    = closestEquivalentAngle(tangent,       podAngle);
            double optB    = closestEquivalentAngle(tangent + 180, podAngle);
            double target  = Math.abs(wrapDeg(optA - podAngle)) <= Math.abs(wrapDeg(optB - podAngle))
                    ? optA : optB;

            targetAngle = target;
            double raw  = SwerveDrive.PodControl.kP * wrapDeg(target - podAngle);
            servo.setPower(Double.isNaN(raw) ? 0 : raw);
        } else {
            targetAngle = currentAngle;
            servo.setPower(0);
        }

        return 0;
    }

    public void set(double power) { motor.setPower(power); }

    private double readAngle() {
        return wrapDeg(Zeroing.polarQuadrature(motor.getPosition()));
    }

    private void trackVelocity(double podAngle) {
        long   now      = System.nanoTime();
        double dt       = Math.max(0.005, Math.min(0.05, (now - lastTime) / 1e9));
        double rawOmega = wrapDeg(podAngle - lastPodAngle) / dt;

        double alpha    = SwerveDrive.PodControl.velocityFilter;
        angularVelocity = alpha * rawOmega + (1.0 - alpha) * angularVelocity;

        lastPodAngle = podAngle;
        lastTime     = now;
    }

    private void commandServo(double angleSetpoint, double predictedAngle, double podAngle) {

        long   now        = System.nanoTime();
        double dtFF       = Math.max(0.005, Math.min(0.05, (now - lastSetpointTime) / 1e9));
        double setpointVelocity = wrapDeg(angleSetpoint - lastAngleSetpoint) / dtFF;
        lastAngleSetpoint = angleSetpoint;
        lastSetpointTime  = now;

        final double kP        = SwerveDrive.PodControl.kP;
        final double kD        = SwerveDrive.PodControl.kD;
        final double kV        = SwerveDrive.PodControl.kV;
        final double kS        = SwerveDrive.PodControl.kS;
        final double deadband  = SwerveDrive.PodControl.errorDeadband;
        final double limitband = SwerveDrive.PodControl.limitband;
        final double limit     = SwerveDrive.PodControl.outputLimit;

        double absError = Math.abs(wrapDeg(angleSetpoint - podAngle));
        double raw;

        if (absError < deadband) {
            raw = 0;
        } else {
            double error    = wrapDeg(angleSetpoint - predictedAngle);
            double softSign = Math.tanh(error / SwerveDrive.PodControl.kSTransition);
            raw  = kV * setpointVelocity + kP * error - kD * angularVelocity;
            raw += softSign * kS * TaskOpMode.Runtime.voltageCompensation;
            if (absError < limitband) {
                raw = Math.max(-limit, Math.min(limit, raw));
            }
        }

        servo.setPower(Double.isNaN(raw) ? 0 : raw);
    }

    private static double wrapDeg(double angle) {
        angle = ((angle % 360) + 360) % 360;
        if (angle > 180) angle -= 360;
        return angle;
    }

    private static double closestEquivalentAngle(double target, double reference) {
        return reference + wrapDeg(target - reference);
    }
}
