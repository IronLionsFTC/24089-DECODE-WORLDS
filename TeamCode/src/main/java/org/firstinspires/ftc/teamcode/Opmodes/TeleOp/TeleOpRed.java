package org.firstinspires.ftc.teamcode.Opmodes.TeleOp;

import org.firstinspires.ftc.teamcode.lioncore.math.types.Position;
import org.firstinspires.ftc.teamcode.lioncore.tasks.Forever;
import org.firstinspires.ftc.teamcode.lioncore.tasks.Jobs;
import org.firstinspires.ftc.teamcode.lioncore.tasks.TaskOpMode;
import org.firstinspires.ftc.teamcode.systems.Intake;
import org.firstinspires.ftc.teamcode.systems.Shooter;
import org.firstinspires.ftc.teamcode.systems.SwerveDrive;
import org.firstinspires.ftc.teamcode.tasks.Reloca;
import org.firstinspires.ftc.teamcode.tasks.Shoot;
import org.firstinspires.ftc.teamcode.tasks.TeleopDriveVector;
import org.firstinspires.ftc.teamcode.tasks.TeleopIntake;

@com.qualcomm.robotcore.eventloop.opmode.TeleOp(name = "TeleOpRed")
public class TeleOpRed extends TaskOpMode {

    @Override
    public Jobs spawn() {

        SwerveDrive drivetrain = new SwerveDrive(
            new Position(-3500, 0, 0),
            controller1.rightJoystick::x,
            true
        );

        Intake intake = new Intake();
        intake.loadHardware(hardwareMap);
        Shooter shooter = new Shooter(intake.yieldTurretEncoder());

        controller1.X.onPress(new TeleopIntake(intake));
        controller1.A.onPress(new Reloca(drivetrain));
        controller1.rightTrigger.asButton.onPress(new Shoot(intake, shooter));

        return Jobs.create()
                .addTask(new Forever(new TeleopDriveVector(
                        drivetrain,
                        () -> gamepad1.left_stick_x,
                        () -> -gamepad1.left_stick_y
                )))
                .registerSystem(shooter)
                .registerSystem(intake)
                .registerSystem(drivetrain);

    }
}
