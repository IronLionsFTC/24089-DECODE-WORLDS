package org.firstinspires.ftc.teamcode.Opmodes.TeleOp;

import org.firstinspires.ftc.teamcode.lioncore.math.types.Position;
import org.firstinspires.ftc.teamcode.lioncore.tasks.Forever;
import org.firstinspires.ftc.teamcode.lioncore.tasks.Jobs;
import org.firstinspires.ftc.teamcode.lioncore.tasks.TaskOpMode;
import org.firstinspires.ftc.teamcode.systems.Indicator;
import org.firstinspires.ftc.teamcode.systems.Intake;
import org.firstinspires.ftc.teamcode.systems.Limelight;
import org.firstinspires.ftc.teamcode.systems.Shooter;
import org.firstinspires.ftc.teamcode.systems.SwerveDrive;
import org.firstinspires.ftc.teamcode.tasks.EndXPattern;
import org.firstinspires.ftc.teamcode.tasks.IndicatorManager;
import org.firstinspires.ftc.teamcode.tasks.LimelightRelocalise;
import org.firstinspires.ftc.teamcode.tasks.Reloca;
import org.firstinspires.ftc.teamcode.tasks.RelocaliseTo;
import org.firstinspires.ftc.teamcode.tasks.Shoot;
import org.firstinspires.ftc.teamcode.tasks.ShootSlow;
import org.firstinspires.ftc.teamcode.tasks.StartXPattern;
import org.firstinspires.ftc.teamcode.tasks.TeleopDriveVector;
import org.firstinspires.ftc.teamcode.tasks.TeleopIntake;

@com.qualcomm.robotcore.eventloop.opmode.TeleOp(name = "TeleOpBlue")
public class TeleOpBlue extends TaskOpMode {

    @Override
    public Jobs spawn() {

        SwerveDrive drivetrain = new SwerveDrive(
            new Position(3500, 3100, 0),
            controller1.rightJoystick::x,
            false
        );

        Intake intake = new Intake();
        intake.loadHardware(hardwareMap);
        Shooter shooter = new Shooter(intake.yieldTurretEncoder());
        Limelight limelight = new Limelight();

        controller1.X.onPress(new TeleopIntake(intake));
        controller1.rightTrigger.asButton.onPress(new Shoot(intake, shooter));
        controller1.leftTrigger.asButton.onPress(new StartXPattern(drivetrain));
        controller1.leftTrigger.asButton.onRelease(new EndXPattern(drivetrain));

        controller1.dpad.left.onPress(new Reloca(drivetrain));
        controller1.dpad.up.onPress(new LimelightRelocalise(drivetrain, limelight));
        controller1.dpad.right.onPress(new RelocaliseTo(drivetrain, new Position(500, 0, 90)));

        controller1.bumpers.right.onPress(new ShootSlow(intake, shooter));

        return Jobs.create()
                .addTask(
                    new Forever(
                        new TeleopDriveVector(
                            drivetrain,
                            () -> gamepad1.left_stick_x,
                            () -> -gamepad1.left_stick_y
                        )
                    )
                )
                .registerSystem(shooter)
                .registerSystem(intake)
                .registerSystem(limelight)
                .registerSystem(drivetrain);

    }
}
