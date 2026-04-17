package org.firstinspires.ftc.teamcode.Opmodes.TeleOp;

import org.firstinspires.ftc.teamcode.lioncore.math.types.Position;
import org.firstinspires.ftc.teamcode.lioncore.paths.Line;
import org.firstinspires.ftc.teamcode.lioncore.tasks.Forever;
import org.firstinspires.ftc.teamcode.lioncore.tasks.Jobs;
import org.firstinspires.ftc.teamcode.lioncore.tasks.Run;
import org.firstinspires.ftc.teamcode.lioncore.tasks.TaskOpMode;
import org.firstinspires.ftc.teamcode.systems.Follower;
import org.firstinspires.ftc.teamcode.systems.Intake;
import org.firstinspires.ftc.teamcode.systems.Limelight;
import org.firstinspires.ftc.teamcode.systems.Shooter;
import org.firstinspires.ftc.teamcode.tasks.EndXPattern;
import org.firstinspires.ftc.teamcode.tasks.Follow;
import org.firstinspires.ftc.teamcode.tasks.Goto;
import org.firstinspires.ftc.teamcode.tasks.Jetison;
import org.firstinspires.ftc.teamcode.tasks.LimelightRelocalise;
import org.firstinspires.ftc.teamcode.tasks.RelocaliseToStart;
import org.firstinspires.ftc.teamcode.tasks.RelocaliseTo;
import org.firstinspires.ftc.teamcode.tasks.Shoot;
import org.firstinspires.ftc.teamcode.tasks.StartXPattern;
import org.firstinspires.ftc.teamcode.tasks.TeleopDriveVector;
import org.firstinspires.ftc.teamcode.tasks.TeleopIntake;

@com.qualcomm.robotcore.eventloop.opmode.TeleOp(name = "TeleOpBlue")
public class TeleOpBlue extends TaskOpMode {

    @Override
    public Jobs spawn() {

        Follower drivetrain = new Follower(
            3500, 3100, 0,
            controller1.rightJoystick::x
        );

        Intake intake = new Intake();
        intake.loadHardware(hardwareMap);
        Shooter shooter = new Shooter(intake.yieldTurretEncoder());
        Limelight limelight = new Limelight(Limelight.Team.Blue);

        double xOffset = -250;
        double yOffset = 150;
        Position gateIntake = new Position(1920 + xOffset, -100 + yOffset, 150);
        Position gateIntakeB = new Position(2020 + xOffset, -100 + yOffset, 150);

        controller1.leftTrigger.asButton.onPress(new TeleopIntake(intake));
        controller1.rightTrigger.asButton.onPress(new Shoot(intake, shooter));
        controller1.X.onPress(new StartXPattern(drivetrain));
        controller1.X.onRelease(new EndXPattern(drivetrain));
        controller1.A.onPress(new Run(() -> drivetrain.setHeading(155)));
        controller1.bumpers.right.onPress(
                new Goto(drivetrain, gateIntake).setMaxSpeed(900).with(
                        new Run(() -> intake.setState(Intake.State.IntakingEmpty))
                ).then(
                        new Follow(drivetrain, new Line(
                                gateIntake,
                                gateIntakeB
                        ))
                )
        );

        controller1.dpad.left.onPress(new RelocaliseToStart(drivetrain));
        controller1.dpad.up.onPress(new LimelightRelocalise(drivetrain, limelight));
        controller1.dpad.right.onPress(new RelocaliseTo(drivetrain, new Position(500, 0, 90)));
        controller1.bumpers.left.onPress(new Jetison(intake));

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
