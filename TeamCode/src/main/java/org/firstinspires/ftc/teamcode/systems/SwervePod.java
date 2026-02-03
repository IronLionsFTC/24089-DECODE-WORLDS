package org.firstinspires.ftc.teamcode.systems;

import org.firstinspires.ftc.teamcode.lioncore.hardware.LionMotor;
import org.firstinspires.ftc.teamcode.lioncore.hardware.LionServo;
import org.firstinspires.ftc.teamcode.lioncore.math.types.Vector;
import org.firstinspires.ftc.teamcode.parameters.ServoConstants;

public class SwervePod {

    private LionMotor motor;
    private LionCRServo servo;
    private Vector offset;

    /**
     *
     * @param motor
     * @param servo
     * @param offset X (right) and y (forward) position of the pod from the center of the robot
     */
    public SwervePod(LionMotor motor, LionServo servo, Vector offset) {
        this.motor = motor;
        this.servo = servo;
        this.offset = offset;
    }

    /**
     * Optimise the movement of a swerve pod to the position it is suppose to be at to require the least amount of movement.
     * @param target A vector representing the direction the pod should go, with +y as forward and +x as right.
     */
    public double update(Vector target, double heading) {

        if (target.magnitude() == 0 && heading == 0) {
            double angle = this.offset.polarDirection();
            servo.setPosition(angle / (255.0 * ServoConstants.Ratios.swerve) + 0.5);
            return 0.0;
        }

        double current = servo.getPosition();
        double currentDegrees = (current - 0.5) * ServoConstants.Ratios.swerve * 255.0;

        // Normalise option A
        Vector rotationalVelocity = Vector.cartesian(heading * offset.y(), -heading * offset.x());
        Vector finalVelocity = rotationalVelocity.add(target);
        double forward = finalVelocity.polarDirection();

        // Normalise option B
        double reversed = forward + 180;
        while (reversed > 180) reversed -= 360;
        while (reversed < -180) reversed += 360;

        double forwardError = Math.abs(forward - currentDegrees);
        double reversedError = Math.abs(reversed - currentDegrees);

        if (forwardError >= reversedError) {
            servo.setPosition(reversed / (255.0 * ServoConstants.Ratios.swerve) + 0.5);
            return -finalVelocity.magnitude();
        } else {
            servo.setPosition(forward / (255.0 * ServoConstants.Ratios.swerve) + 0.5);
            return finalVelocity.magnitude();
        }
    }

    public void set(double power) {
        this.motor.setPower(power);
    }
}
