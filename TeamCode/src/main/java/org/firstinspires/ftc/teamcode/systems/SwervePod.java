package org.firstinspires.ftc.teamcode.systems;

import org.firstinspires.ftc.teamcode.lioncore.hardware.LionCRServo;
import org.firstinspires.ftc.teamcode.lioncore.hardware.LionMotor;
import org.firstinspires.ftc.teamcode.lioncore.math.pid.PID;
import org.firstinspires.ftc.teamcode.lioncore.math.types.Vector2;
import org.firstinspires.ftc.teamcode.parameters.Zeroing;

public class SwervePod {

    private final LionMotor motor;
    private final LionCRServo servo;
    private final Vector2 offset;

    private boolean xPattern;
    private final PID angleController;
    private double lastAngleSetpoint;

    public SwervePod(
            LionMotor motor,
            LionCRServo servo,
            Vector2 offset,
            double startDegrees,
            boolean xPattern
    ) {
        this.motor = motor;
        this.servo = servo;
        this.offset = offset;
        this.xPattern = xPattern;

        motor.resetPositionTo(startDegrees * (4096.0 / 360.0));

        this.angleController = new PID(0,0,0);
        this.lastAngleSetpoint = startDegrees;
    }

    public double update(Vector2 translation, double omega) {

        double podAngle = wrapDeg(Zeroing.polarQuadrature(motor.getPosition()));

        angleController.setConstants(
                SwerveDrive.SwervePID.P,
                SwerveDrive.SwervePID.I,
                SwerveDrive.SwervePID.D
        );

        double translationMag = translation.magnitude();
        double omegaMag = Math.abs(omega);

        boolean commandedIdle = translationMag < 1e-6 && omegaMag < 1e-6;

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

                servo.setPower(angleController.calculate(podAngle, target));

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

        servo.setPower(angleController.calculate(podAngle, angleSetpoint));

        if (Math.abs(wrapDeg(angleSetpoint - podAngle)) > 40) {
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
