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

    /**
     *
     * @param motor
     * @param servo
     * @param offset X (right) and y (forward) position of the pod from the center of the robot
     */
    public SwervePod(LionMotor motor, LionCRServo servo, Vector offset, double startDegrees) {
        this.motor = motor;
        this.servo = servo;
        this.offset = offset;
        this.motor.resetPositionTo(startDegrees * (4096.0 / 360.0));
        this.angleController = new PID(0, 0, 0);
    }

    /**
     * Optimise the movement of a swerve pod to the position it is suppose to be at to require the least amount of movement.
     * @param target A vector representing the direction the pod should go, with +y as forward and +x as right.
     */
    public double update(Vector target, double heading) {

        this.podAngle = Zeroing.polarQuadrature(this.motor.getPosition());

        this.angleController.setConstants(
                SwerveDrive.SwervePID.P,
                SwerveDrive.SwervePID.I,
                SwerveDrive.SwervePID.D
        );

        if (target.magnitude() == 0 && heading == 0) {
            double angle = this.offset.polarDirection();
            servo.setPower(angleController.calculate(podAngle, angle));
            return 0.0;
        }

        // Normalise option A
        Vector rotationalVelocity = Vector.cartesian(heading * offset.y(), -heading * offset.x());
        Vector finalVelocity = rotationalVelocity.add(target);
        double forward = finalVelocity.polarDirection();

        // Normalise option B
        double reversed = forward + 180;
        while (reversed > 180) reversed -= 360;
        while (reversed < -180) reversed += 360;

        double forwardError = ((forward - podAngle + 180) % 360) - 180;
        double reversedError = ((reversed - podAngle + 180) % 360) - 180;

        if (Math.abs(forwardError) > Math.abs(reversedError)) {
            double response = angleController.calculate(forwardError, 0);
            this.servo.setPower(response);
            return finalVelocity.magnitude();
        } else {
            double response = angleController.calculate(reversedError, 0);
            this.servo.setPower(response);
            return -finalVelocity.magnitude();
        }
    }

    public void set(double power) {
        this.motor.setPower(power);
    }
}
