package org.firstinspires.ftc.teamcode.Opmodes.TeleOp;

import org.firstinspires.ftc.teamcode.lioncore.math.types.Position;
import org.firstinspires.ftc.teamcode.lioncore.tasks.Jobs;
import org.firstinspires.ftc.teamcode.lioncore.tasks.TaskOpMode;
import org.firstinspires.ftc.teamcode.systems.SwerveDrive;

@com.qualcomm.robotcore.eventloop.opmode.TeleOp
public class TeleOp extends TaskOpMode {

    @Override
    public Jobs spawn() {

        SwerveDrive drivetrain = new SwerveDrive(
            new Position(0, 0, 0),
            () -> gamepad1.left_stick_x,
            () -> -gamepad1.left_stick_y,
            () -> gamepad1.right_stick_x * 0.6
        );

        return Jobs.create()
                .registerSystem(drivetrain);

    }
}
