package org.firstinspires.ftc.teamcode.systems;

import org.firstinspires.ftc.teamcode.lioncore.hardware.LionCRServo;
import org.firstinspires.ftc.teamcode.lioncore.hardware.LionMotor;
import org.firstinspires.ftc.teamcode.lioncore.math.pid.PID;
import org.firstinspires.ftc.teamcode.lioncore.math.types.Vector2;
import org.firstinspires.ftc.teamcode.lioncore.tasks.TaskOpMode;
import org.firstinspires.ftc.teamcode.parameters.Zeroing;

public class SwervePod {

    private final LionMotor motor;
    private final LionCRServo servo;
    private final Vector2 offset;

    private boolean xPattern;
    private double lastAngleSetpoint;
    private final SwerveDrive.Pod pod;

    public double currentAngle = 0;
    public double targetAngle = 0;

    private double lastPodAngle = 0;
    private long lastTime = System.nanoTime();
    private double angularVelocity = 0; // deg/sec

    public SwervePod(
            LionMotor motor,
            LionCRServo servo,
            Vector2 offset,
            double startDegrees,
            boolean xPattern,
            SwerveDrive.Pod pod
    ) {
        this.pod = pod;
        this.motor = motor;
        this.servo = servo;
        this.offset = offset;
        this.xPattern = xPattern;

        motor.resetPositionTo(startDegrees * (4096.0 / 360.0));

        this.lastAngleSetpoint = startDegrees;
    }

    public double update(Vector2 translation, double omega) {

        double podAngle = wrapDeg(Zeroing.polarQuadrature(motor.getPosition()));
        long now = System.nanoTime();
        double dt = (now - lastTime) / 1e9;

        // Clamp dt to avoid spikes (VERY important in FTC)
        dt = Math.max(0.005, Math.min(0.05, dt));

        // Compute wrapped delta
        double delta = wrapDeg(podAngle - lastPodAngle);

        // Angular velocity (deg/sec)
        angularVelocity = delta / dt;
        double predictedAngle = podAngle + angularVelocity * SwerveDrive.SwervePID.latency;

        // Update history
        lastPodAngle = podAngle;
        lastTime = now;

        double p;
        double d;
        double kS;

        switch (this.pod) {
            case Fr:
                p = SwerveDrive.SwervePID.Pfr;
                d = SwerveDrive.SwervePID.Dfr;
                kS = SwerveDrive.SwervePID.kSfr;
                break;
            case Fl:
                p = SwerveDrive.SwervePID.Pfl;
                d = SwerveDrive.SwervePID.Dfl;
                kS = SwerveDrive.SwervePID.kSfl;
                break;
            case Rr:
                p = SwerveDrive.SwervePID.Prr;
                d = SwerveDrive.SwervePID.Drr;
                kS = SwerveDrive.SwervePID.kSrr;
                break;
            case Rl:
                p = SwerveDrive.SwervePID.Prl;
                d = SwerveDrive.SwervePID.Drl;
                kS = SwerveDrive.SwervePID.kSrl;
                break;
            default:
                p = 0.009;
                d = 0;
                kS = 0;
        }

        double translationMag = translation.magnitude();
        double omegaMag = Math.abs(omega);

        boolean commandedIdle = translationMag < 0.1 && omegaMag < 0.1;

        if (commandedIdle) {

            if (xPattern) {

                double optionA = closestEquivalentAngle(
                        wrapDeg(offset.polarDirection() + 90),
                        podAngle
                );

                double optionB = closestEquivalentAngle(
                        wrapDeg(offset.polarDirection() - 90),
                        podAngle
                );

                double target = Math.abs(wrapDeg(optionA - podAngle))
                        < Math.abs(wrapDeg(optionB - podAngle))
                        ? optionA
                        : optionB;

                double response = (target - podAngle) * p;
                if (!Double.isNaN(response)) servo.setPower(response);
                else servo.setPower(0);

                lastAngleSetpoint = target;

            } else {
                servo.setPower(0);
            }

            return 0;
        }

        Vector2 rotationalVelocity = Vector2.cartesian(
                omega * offset.y(),
                -omega * offset.x()
        );

        Vector2 finalVelocity = translation.add(rotationalVelocity);

        if (finalVelocity.magnitude() < 1e-6) {
            servo.setPower(0);
            return 0;
        }

        double rawDesired = -finalVelocity.polarDirection();

        double forwardAngle = closestEquivalentAngle(rawDesired, podAngle);
        double reversedAngle = closestEquivalentAngle(rawDesired + 180, podAngle);

        double forwardError = Math.abs(wrapDeg(forwardAngle - podAngle));
        double reversedError = Math.abs(wrapDeg(reversedAngle - podAngle));

        double angleSetpoint;
        double drivePower;

        if (forwardError <= reversedError) {
            angleSetpoint = forwardAngle;
            drivePower = finalVelocity.magnitude();
        } else {
            angleSetpoint = reversedAngle;
            drivePower = -finalVelocity.magnitude();
        }

        lastAngleSetpoint = angleSetpoint;

        this.currentAngle = podAngle;
        this.targetAngle = angleSetpoint;

        double error = wrapDeg(angleSetpoint - predictedAngle);

        // PD with derivative on measurement
        double raw =
                p * error
                - d * angularVelocity;

        raw += Math.signum(error) * kS * TaskOpMode.Runtime.voltageCompensation;
        if (Math.abs(podAngle - angleSetpoint) < SwerveDrive.SwervePID.deadband) raw = 0;
        if (Math.abs(podAngle - angleSetpoint) < SwerveDrive.SwervePID.limitband) {
            if (raw < -SwerveDrive.SwervePID.limit) raw = -SwerveDrive.SwervePID.limit;
            if (raw >  SwerveDrive.SwervePID.limit) raw =  SwerveDrive.SwervePID.limit;
        }

        if (!Double.isNaN(raw)) servo.setPower(raw);
        else servo.setPower(0);

        if (Math.abs(wrapDeg(angleSetpoint - podAngle)) > 20) {
            return 0;
        }

        return drivePower;
    }

    public void set(double power) {
        motor.setPower(power);
    }

    public void setXPattern(boolean allowXPattern) {
        this.xPattern = allowXPattern;
    }

    private static double wrapDeg(double angle) {
        angle %= 360;

        if (angle <= -180) angle += 360;
        if (angle > 180) angle -= 360;

        return angle;
    }

    private static double closestEquivalentAngle(double target, double reference) {
        return reference + wrapDeg(target - reference);
    }
}
