package org.firstinspires.ftc.teamcode.Opmodes.TeleOp;

import org.firstinspires.ftc.teamcode.lioncore.math.types.Position;
import org.firstinspires.ftc.teamcode.lioncore.tasks.Forever;
import org.firstinspires.ftc.teamcode.lioncore.tasks.Jobs;
import org.firstinspires.ftc.teamcode.lioncore.tasks.Run;
import org.firstinspires.ftc.teamcode.lioncore.tasks.TaskOpMode;
import org.firstinspires.ftc.teamcode.systems.Follower;
import org.firstinspires.ftc.teamcode.systems.Intake;
import org.firstinspires.ftc.teamcode.systems.Limelight;
import org.firstinspires.ftc.teamcode.systems.Shooter;
import org.firstinspires.ftc.teamcode.tasks.DownAdjust;
import org.firstinspires.ftc.teamcode.tasks.EndXPattern;
import org.firstinspires.ftc.teamcode.tasks.IntakeUntilFull;
import org.firstinspires.ftc.teamcode.tasks.Jetison;
import org.firstinspires.ftc.teamcode.tasks.LimelightTrack;
import org.firstinspires.ftc.teamcode.tasks.RelocaliseTo;
import org.firstinspires.ftc.teamcode.tasks.Shoot;
import org.firstinspires.ftc.teamcode.tasks.StartXPattern;
import org.firstinspires.ftc.teamcode.tasks.TeleopDriveVector;
import org.firstinspires.ftc.teamcode.tasks.TeleopIntake;
import org.firstinspires.ftc.teamcode.tasks.ToggleSOTM;
import org.firstinspires.ftc.teamcode.tasks.UpAdjust;

@com.qualcomm.robotcore.eventloop.opmode.TeleOp(name = "TeleOpBlue")
public class TeleOpBlue extends TaskOpMode {

    @Override
    public Jobs spawn() {

        Follower drivetrain = new Follower(
            3300, 3150, 0,
            controller1.rightJoystick::x
        );

        Intake intake = new Intake();
        intake.loadHardware(hardwareMap);
        Shooter shooter = new Shooter(intake.yieldTurretEncoder());

        controller1.leftTrigger.asButton.onPress(new TeleopIntake(intake));
        controller1.rightTrigger.asButton.onPress(new Shoot(intake, shooter));
        controller1.X.onPress(new StartXPattern(drivetrain));
        controller1.X.onRelease(new EndXPattern(drivetrain));
        controller1.A.onPress(new Run(() -> drivetrain.setHeading(150)));
        Limelight limelight = new Limelight();

        controller1.Y.onPress(new ToggleSOTM());

        controller2.dpad.up.onPress(new UpAdjust(shooter));
        controller2.dpad.down.onPress(new DownAdjust(shooter));
        controller1.dpad.left.onPress(new RelocaliseTo(drivetrain, new Position(3150, -3300, 0)));
        controller1.dpad.right.onPress(new RelocaliseTo(drivetrain, new Position(500, 0, 90)));
        controller1.bumpers.left.onPress(new Jetison(intake));

        return Jobs.create()
                .addSeries(
                    new Run(() -> Shooter.ShooterPID.useConvergence = true),
                    new Forever(
                        new TeleopDriveVector(
                            drivetrain,
                            () -> gamepad1.left_stick_x,
                            () -> -gamepad1.left_stick_y
                        )
                    )
                )
                .registerSystem(limelight)
                .registerSystem(shooter)
                .registerSystem(intake)
                .registerSystem(drivetrain);

    }
}
