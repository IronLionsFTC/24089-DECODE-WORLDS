package org.firstinspires.ftc.teamcode.Opmodes.Debug;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.lioncore.math.types.Position;
import org.firstinspires.ftc.teamcode.lioncore.paths.Line;
import org.firstinspires.ftc.teamcode.lioncore.tasks.Jobs;
import org.firstinspires.ftc.teamcode.lioncore.tasks.Run;
import org.firstinspires.ftc.teamcode.lioncore.tasks.Sleep;
import org.firstinspires.ftc.teamcode.lioncore.tasks.TaskOpMode;
import org.firstinspires.ftc.teamcode.lioncore.tasks.WaitUntil;
import org.firstinspires.ftc.teamcode.systems.Follower;
import org.firstinspires.ftc.teamcode.systems.Intake;
import org.firstinspires.ftc.teamcode.systems.Limelight;
import org.firstinspires.ftc.teamcode.systems.Shooter;
import org.firstinspires.ftc.teamcode.tasks.AutoLimelightTrack;
import org.firstinspires.ftc.teamcode.tasks.Follow;
import org.firstinspires.ftc.teamcode.tasks.Shoot;

@Autonomous
public class CVTest extends TaskOpMode {
    @Override
    public Jobs spawn() {

        double xOffset = 100;
        double yOffset = 0;

        Limelight limelight = new Limelight();
        Follower follower = new Follower(3200 + xOffset, 1200 + yOffset, 180);
        Intake intake = new Intake();
        intake.loadHardware(hardwareMap);
        Shooter shooter = new Shooter(intake.yieldTurretEncoder());

        Position shoot = new Position(3000 + xOffset, 1200 + yOffset, 180);
        Position wallIntakeA = new Position(3190 + xOffset, 0 + yOffset, 180);

        return Jobs.create()
                .addSeries(
                        new Run(() -> Shooter.ShooterPID.useConvergence = false),
                        new WaitUntil(shooter::atSpeed),
                        new Sleep(0.3),

                        // WALL CYCLE
                        new AutoLimelightTrack(follower, limelight).then(
                                new Follow(follower, new Line(wallIntakeA, shoot)).setMaxSpeed(1100)
                        ).with(
                                new Run(() -> intake.setState(Intake.State.IntakingEmpty))
                        ),
                        new Sleep(0.5),
                        new Shoot(intake, shooter),

                        // WALL CYCLE
                        new AutoLimelightTrack(follower, limelight).then(
                                new Follow(follower, new Line(wallIntakeA, shoot)).setMaxSpeed(1100)
                        ).with(
                                new Run(() -> intake.setState(Intake.State.IntakingEmpty))
                        ),
                        new Sleep(0.5),
                        new Shoot(intake, shooter),

                        // WALL CYCLE
                        new AutoLimelightTrack(follower, limelight).then(
                                new Follow(follower, new Line(wallIntakeA, shoot)).setMaxSpeed(1100)
                        ).with(
                                new Run(() -> intake.setState(Intake.State.IntakingEmpty))
                        ),
                        new Sleep(0.5),
                        new Shoot(intake, shooter),

                        // WALL CYCLE
                        new AutoLimelightTrack(follower, limelight).then(
                                new Follow(follower, new Line(wallIntakeA, shoot)).setMaxSpeed(1100)
                        ).with(
                                new Run(() -> intake.setState(Intake.State.IntakingEmpty))
                        ),
                        new Sleep(0.5),
                        new Shoot(intake, shooter)
                )
                .registerSystem(limelight)
                .registerSystem(shooter)
                .registerSystem(intake)
                .registerSystem(follower);

    }
}
