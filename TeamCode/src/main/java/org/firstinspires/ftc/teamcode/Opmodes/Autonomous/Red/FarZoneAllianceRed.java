package org.firstinspires.ftc.teamcode.Opmodes.Autonomous.Red;

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
import org.firstinspires.ftc.teamcode.tasks.Goto;
import org.firstinspires.ftc.teamcode.tasks.IntakeUntilFull;
import org.firstinspires.ftc.teamcode.tasks.Shoot;

@Autonomous
public class FarZoneAllianceRed extends TaskOpMode {
    @Override
    public Jobs spawn() {

        double xOffset = -100;
        double yOffset = 0;

        Limelight limelight = new Limelight();
        Follower follower = new Follower(-3200 + xOffset, 1200 + yOffset, 180);
        Intake intake = new Intake();
        intake.loadHardware(hardwareMap);
        Shooter shooter = new Shooter(intake.yieldTurretEncoder());

        Position start = new Position(-3200 + xOffset, 1200 + yOffset, 180);
        Position shoot = new Position(-3000 + xOffset, 1200 + yOffset, 180);

        Position wallIntakeA = new Position(-3190 + xOffset, 0 + yOffset, 180);
        Position intakeAStart = new Position(-2500 + xOffset, 1000 + yOffset, 180);
        Position intakeAEnd = new Position(-2500 + xOffset, -50 + yOffset, 180);
        Position intakeBStart = new Position(-1900 + xOffset, 900 + yOffset, 180);
        Position intakeBEnd = new Position(-1900 + xOffset, -50 + yOffset, 180);

        return Jobs.create()
                .addSeries(
                        new Run(() -> Shooter.ShooterPID.useConvergence = false),
                        new WaitUntil(shooter::atSpeed),
                        new Sleep(0.3),
                        new Shoot(intake, shooter),

                        new Follow(follower, new Line(
                                shoot, wallIntakeA
                        )).setMaxSpeed(1000).race(
                                new IntakeUntilFull(intake)
                        ).then(
                                new Sleep(0.4).then(
                                        new Follow(follower, new Line(
                                                wallIntakeA,
                                                shoot
                                        )).setMaxSpeed(1300).master(
                                                new IntakeUntilFull(intake)
                                        ))
                        ),
                        new Sleep(0.4),
                        new Shoot(intake, shooter),
                        new Follow(follower, new Line(
                                shoot,
                                intakeAStart
                        )).setMaxSpeed(900).then(
                                new Sleep(0.2).then(
                                        new Follow(follower, new Line(
                                                intakeAStart,
                                                intakeAEnd
                                        )).setMaxSpeed(900))
                        ).race(
                                new IntakeUntilFull(intake)
                        ),
                        new Follow(follower, new Line(
                                intakeAEnd,
                                shoot
                        )).setMaxSpeed(900),
                        new Sleep(0.4),
                        new Shoot(intake, shooter),

                        // WALL CYCLE
                        new AutoLimelightTrack(follower, limelight).then(
                                new Follow(follower, new Line(wallIntakeA, shoot)).setMaxSpeed(1000)
                        ).with(
                                new Run(() -> intake.setState(Intake.State.IntakingEmpty))
                        ),
                        new Sleep(0.5),
                        new Shoot(intake, shooter),

                        // WALL CYCLE
                        new AutoLimelightTrack(follower, limelight).then(
                                new Follow(follower, new Line(wallIntakeA, shoot)).setMaxSpeed(1000)
                        ).with(
                                new Run(() -> intake.setState(Intake.State.IntakingEmpty))
                        ),
                        new Sleep(0.5),
                        new Shoot(intake, shooter),

                        // WALL CYCLE
                        new AutoLimelightTrack(follower, limelight).then(
                                new Follow(follower, new Line(wallIntakeA, shoot)).setMaxSpeed(1000)
                        ).with(
                                new Run(() -> intake.setState(Intake.State.IntakingEmpty))
                        ),
                        new Sleep(0.5),
                        new Shoot(intake, shooter),

                        // WALL CYCLE
                        new AutoLimelightTrack(follower, limelight).then(
                                new Follow(follower, new Line(wallIntakeA, shoot)).setMaxSpeed(1000)
                        ).with(
                                new Run(() -> intake.setState(Intake.State.IntakingEmpty))
                        ),
                        new Sleep(0.5),
                        new Shoot(intake, shooter),

                        new Follow(follower, new Line(shoot, intakeAStart))
                )
                .registerSystem(limelight)
                .registerSystem(shooter)
                .registerSystem(intake)
                .registerSystem(follower);

    }
}
