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
    public void update(Vector target) {

        double current = servo.getPosition() * (21.0 / 16.0);
        double currentDegrees = (current - 0.5) * ServoConstants.Ratios.swerve * 360;

        // Normalise option A
        double forward = 90 - Math.toDegrees(target.direction());
        while (forward > 180) forward -= 360;
        while (forward < -180) forward += 360;

        // Normalise option B
        double reversed = Math.toDegrees(target.direction()) + 180;
        while (reversed > 180) reversed -= 360;
        while (reversed < -180) reversed += 360;

        // Normalise current direction
        while (currentDegrees > 180) currentDegrees -= 360;
        while (currentDegrees < -180) currentDegrees += 360;

        double forwardError = Math.abs(forward - current);
        double reversedError = Math.abs(reversed - current);

        if (forwardError >= reversedError) {
            servo.setPosition(reversed / (360 * ServoConstants.Ratios.swerve) + 0.5);
            motor.setPower(-target.magnitude());
        } else {
            servo.setPosition(forward / (360 * ServoConstants.Ratios.swerve) + 0.5);
            motor.setPower(target.magnitude());
        }
    }
}
