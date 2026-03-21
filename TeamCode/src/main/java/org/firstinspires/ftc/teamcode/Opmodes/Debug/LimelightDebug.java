package org.firstinspires.ftc.teamcode.Opmodes.Debug;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.lioncore.hardware.LimelightProxy;
import org.firstinspires.ftc.teamcode.lioncore.math.types.Position;
import org.firstinspires.ftc.teamcode.lioncore.tasks.Jobs;
import org.firstinspires.ftc.teamcode.lioncore.tasks.TaskOpMode;
import org.firstinspires.ftc.teamcode.systems.Intake;
import org.firstinspires.ftc.teamcode.systems.Limelight;
import org.firstinspires.ftc.teamcode.systems.Shooter;
import org.firstinspires.ftc.teamcode.systems.SwerveDrive;
import org.firstinspires.ftc.teamcode.tasks.LimelightRelocalise;
import org.firstinspires.ftc.teamcode.tasks.Reloca;
import org.firstinspires.ftc.teamcode.tasks.Shoot;
import org.firstinspires.ftc.teamcode.tasks.TeleopIntake;

@TeleOp
public class LimelightDebug extends TaskOpMode {

    public Jobs spawn() {

        SwerveDrive drivetrain = new SwerveDrive(
                new Position(0, 0, 0),
                controller1.rightJoystick::x,
                true
        );

        Intake intake = new Intake();
        intake.loadHardware(hardwareMap);
        Shooter shooter = new Shooter(intake.yieldTurretEncoder());
        Limelight limelight = new Limelight();

        controller1.X.onPress(new TeleopIntake(intake));
        controller1.A.onPress(new Reloca(drivetrain));
        controller1.rightTrigger.asButton.onPress(new Shoot(intake, shooter));
        controller1.Y.onPress(new LimelightRelocalise(drivetrain, limelight));

        return Jobs.create()
                .registerSystem(drivetrain)
                .registerSystem(intake)
                .registerSystem(shooter)
                .registerSystem(limelight);
    }
}
