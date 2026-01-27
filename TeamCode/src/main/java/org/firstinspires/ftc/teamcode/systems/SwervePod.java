package org.firstinspires.ftc.teamcode.systems;

import org.firstinspires.ftc.teamcode.lioncore.hardware.LionMotor;
import org.firstinspires.ftc.teamcode.lioncore.hardware.LionServo;
import org.firstinspires.ftc.teamcode.lioncore.math.types.Vector;
import org.firstinspires.ftc.teamcode.parameters.ServoConstants;

public class SwervePod {

    private LionMotor motor;
    private LionServo servo;

    public SwervePod(LionMotor motor, LionServo servo) {
        this.motor = motor;
        this.servo = servo;
    }

    /**
     * Optimise the movement of a swerve pod to the position it is suppose to be at to require the least amount of movement.
     * @param target A vector representing the direction the pod should go, with +y as forward and +x as right.
     */
    public double update(Vector target) {

        double current = servo.getPosition();
        double currentDegrees = (current - 0.5) * ServoConstants.Ratios.swerve * 255.0;

        // Normalise option A
        double forward = target.polarDirection();

        // Normalise option B
        double reversed = forward + 180;
        while (reversed > 180) reversed -= 360;
        while (reversed < -180) reversed += 360;

        double forwardError = Math.abs(forward - current);
        double reversedError = Math.abs(reversed - current);

        if (forwardError >= reversedError) {
            servo.setPosition(reversed / (255.0 * ServoConstants.Ratios.swerve) + 0.5);
            motor.setPower(-target.magnitude());
        } else {
            servo.setPosition(forward / (255.0 * ServoConstants.Ratios.swerve) + 0.5);
            motor.setPower(target.magnitude());
        }

        return currentDegrees;
    }
}
