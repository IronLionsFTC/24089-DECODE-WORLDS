package org.firstinspires.ftc.teamcode.Opmodes.Autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.lioncore.math.types.Position;
import org.firstinspires.ftc.teamcode.lioncore.paths.Line;
import org.firstinspires.ftc.teamcode.lioncore.tasks.Jobs;
import org.firstinspires.ftc.teamcode.lioncore.tasks.Repeat;
import org.firstinspires.ftc.teamcode.lioncore.tasks.Run;
import org.firstinspires.ftc.teamcode.lioncore.tasks.Series;
import org.firstinspires.ftc.teamcode.lioncore.tasks.Sleep;
import org.firstinspires.ftc.teamcode.lioncore.tasks.TaskOpMode;
import org.firstinspires.ftc.teamcode.lioncore.tasks.WaitUntil;
import org.firstinspires.ftc.teamcode.systems.Follower;
import org.firstinspires.ftc.teamcode.systems.Intake;
import org.firstinspires.ftc.teamcode.systems.Shooter;
import org.firstinspires.ftc.teamcode.tasks.Follow;
import org.firstinspires.ftc.teamcode.tasks.IntakeUntilFull;
import org.firstinspires.ftc.teamcode.tasks.IntakeUntilFullTimeout;
import org.firstinspires.ftc.teamcode.tasks.Shoot;

@Autonomous
public class CloseZoneBlue extends TaskOpMode {
    @Override
    public Jobs spawn() {

        Follower follower = new Follower(0, 800, 180);
        Intake intake = new Intake();
        intake.loadHardware(hardwareMap);
        Shooter shooter = new Shooter(intake.yieldTurretEncoder());

        Position start = new Position(0, 800, 180);
        Position shootA = new Position(1500, 1500, 190);
        Position shootB = new Position(1300, 1400, 160);
        Position intakeAEnd = new Position(2000, 200, 180);
        Position gateIntake = new Position(1900, 250, 160);

        return Jobs.create()
                .addSeries(
                        new Run(() -> Shooter.ShooterPID.targetZClose = 1300),
                        new Follow(follower, new Line(
                                start,
                                shootA
                        )).setMaxSpeed(800).with(
                                new WaitUntil(shooter::atSpeed).then(
                                        new Sleep(0.5).then(new Shoot(intake, shooter))
                                )
                        ),
                        new Run(() -> Shooter.ShooterPID.targetZClose = 1500),
                        new Follow(follower, new Line(
                                shootA, intakeAEnd
                        )).race(
                                new IntakeUntilFull(intake)
                        ),
                        new Follow(follower, new Line(
                                intakeAEnd,
                                shootB
                        )),
                        new Sleep(0.8),
                        new Shoot(intake, shooter),

                                new Series(
                                    new Follow(follower, new Line(
                                        shootB,
                                        gateIntake
                                    )),

                                    new IntakeUntilFullTimeout(intake, 2),

                                    new Follow(follower, new Line(
                                        gateIntake, shootB
                                    )),
                                    new Sleep(0.8),
                                    new Shoot(intake, shooter)
                                )
                    )
                .registerSystem(shooter)
                .registerSystem(intake)
                .registerSystem(follower);

    }
}
