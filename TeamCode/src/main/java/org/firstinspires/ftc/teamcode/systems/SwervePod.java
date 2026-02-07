package org.firstinspires.ftc.teamcode.systems;

import org.firstinspires.ftc.teamcode.lioncore.hardware.LionCRServo;
import org.firstinspires.ftc.teamcode.lioncore.hardware.LionMotor;
import org.firstinspires.ftc.teamcode.lioncore.math.pid.PID;
import org.firstinspires.ftc.teamcode.lioncore.math.types.Vector;
import org.firstinspires.ftc.teamcode.parameters.Zeroing;

public class SwervePod {

    private LionMotor motor;
    private LionCRServo servo;
    private Vector offset;

    private double podAngle;
    private PID angleController;

    private final boolean xPattern;

    /**
     *
     * @param motor
     * @param servo
     * @param offset X (right) and y (forward) position of the pod from the center of the robot
     */
    public SwervePod(LionMotor motor, LionCRServo servo, Vector offset, double startDegrees, boolean xPattern) {
        this.motor = motor;
        this.servo = servo;
        this.offset = offset;
        this.motor.resetPositionTo(startDegrees * (4096.0 / 360.0));
        this.angleController = new PID(0, 0, 0);
        this.xPattern = xPattern;
    }

    /**
     * Updates swerve pod angle and returns drive power.
     *
     * @param target  Desired translation vector (+y forward, +x right)
     * @param heading Desired rotational velocity
     * @return drive motor power (-1 to 1)
     */
    public double update(Vector target, double heading) {

        double podAngle = wrapDeg(Zeroing.polarQuadrature(motor.getPosition()));

        angleController.setConstants(
                SwerveDrive.SwervePID.P,
                SwerveDrive.SwervePID.I,
                SwerveDrive.SwervePID.D
        );

        if (target.magnitude() < 1e-6 && Math.abs(heading) < 1e-6) {

            if (xPattern) {
                double option_a = wrapDeg(closestEquivalentAngle(wrapDeg(offset.polarDirection() + 90), podAngle));
                double option_b = wrapDeg(closestEquivalentAngle(wrapDeg(offset.polarDirection() - 90), podAngle));

                if (Math.abs(podAngle - option_a) <= Math.abs(podAngle - option_b)) {
                    servo.setPower(angleController.calculate(podAngle, option_a));
                } else {
                    servo.setPower(angleController.calculate(podAngle, option_b));
                }
            } else {
                servo.setPower(angleController.calculate(podAngle, podAngle));
            }

            return 0.0;
        }

        Vector rotationalVelocity =
                Vector.cartesian(heading * offset.y(), -heading * offset.x());
        Vector finalVelocity = target.add(rotationalVelocity);

        // Guard against near-zero direction noise
        if (finalVelocity.magnitude() < 1e-6) {
            if (xPattern) {
                double option_a = wrapDeg(closestEquivalentAngle(wrapDeg(offset.polarDirection() + 90), podAngle));
                double option_b = wrapDeg(closestEquivalentAngle(wrapDeg(offset.polarDirection() - 90), podAngle));

                if (Math.abs(podAngle - option_a) <= Math.abs(podAngle - option_b)) {
                    servo.setPower(angleController.calculate(podAngle, option_a));
                } else {
                    servo.setPower(angleController.calculate(podAngle, option_b));
                }
            } else {
                servo.setPower(angleController.calculate(podAngle, podAngle));
            }

            return 0.0;
        }

        double rawDesired = -finalVelocity.polarDirection();

        // Compute both options RELATIVE to current angle
        double forwardAngle  = closestEquivalentAngle(rawDesired, podAngle);
        double reversedAngle = closestEquivalentAngle(rawDesired + 180.0, podAngle);

        double forwardError  = Math.abs(forwardAngle - podAngle);
        double reversedError = Math.abs(reversedAngle - podAngle);

        double angleSetpoint;
        double drivePower;

        if (forwardError <= reversedError) {
            angleSetpoint = forwardAngle;
            drivePower = finalVelocity.magnitude();
        } else {
            angleSetpoint = reversedAngle;
            drivePower = -finalVelocity.magnitude();
        }

        servo.setPower(angleController.calculate(podAngle, angleSetpoint));
        return drivePower;
    }

    public void set(double power) {
        this.motor.setPower(power);
    }

    private static double wrapDeg(double angle) {
        angle %= 360.0;
        if (angle <= -180) angle += 360;
        if (angle > 180) angle -= 360;
        return angle;
    }

    private static double closestEquivalentAngle(double target, double reference) {
        return reference + wrapDeg(target - reference);
    }
}
