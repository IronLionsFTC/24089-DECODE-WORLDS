package org.firstinspires.ftc.teamcode.Opmodes.Autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.lioncore.math.types.Position;
import org.firstinspires.ftc.teamcode.lioncore.paths.Line;
import org.firstinspires.ftc.teamcode.lioncore.tasks.Jobs;
import org.firstinspires.ftc.teamcode.lioncore.tasks.Master;
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
public class CloseZoneGateCycleBlue extends TaskOpMode {
    @Override
    public Jobs spawn() {

        double xOffset = -200;
        double yOffset = 250;

        Follower follower = new Follower(0 + xOffset, 500 + yOffset, 180);
        Intake intake = new Intake();
        intake.loadHardware(hardwareMap);
        Shooter shooter = new Shooter(intake.yieldTurretEncoder());

        Position start = new Position(0 + xOffset, 500 + yOffset, 180);
        Position sotmDest = new Position(1900 + xOffset, 1800 + yOffset, 180);
        Position shootA = new Position(1400 + xOffset, 1300 + yOffset, 150);
        Position shootB = new Position(1300 + xOffset, 1000 + yOffset, 150);
        Position intakeAEnd = new Position(2000 + xOffset, -150 + yOffset, 150);
        Position intakeBEnd = new Position(1350 + xOffset, -60 + yOffset, 150);
        Position gateIntake = new Position(1900 + xOffset, -100 + yOffset, 150);
        Position endPoint = new Position(2000 + xOffset, 300 + yOffset, 150);

        return Jobs.create()
                .addSeries(
                        new Follow(follower, new Line(
                                start,
                                sotmDest
                        )).setMaxSpeed(800).race(
                            new WaitUntil(shooter::atSpeed).then(
                                new Sleep(0.4).then(
                                    new Shoot(intake, shooter)
                                        .then(new Sleep(0.8))
                                )
                            )
                        ),
                        new Follow(follower, new Line(
                                sotmDest, intakeAEnd
                        )).setMaxSpeed(900).race(
                                new IntakeUntilFull(intake)
                        ),
                        new Run(() -> Shooter.ShooterPID.useConvergence = false),
                        new Follow(follower, new Line(
                                intakeAEnd,
                                shootA
                        )),

                        new Shoot(intake, shooter),

                        // GATE CYCLE

                        new Series(
                            new Follow(follower, new Line(
                                shootA,
                                gateIntake
                            )).setMaxSpeed(900),

                            new IntakeUntilFullTimeout(intake, 2),

                            new Follow(follower, new Line(
                                gateIntake, shootA
                            )),
                            new Shoot(intake, shooter)
                        ),

                        // SECOND PREPLACED ROW
                        new Follow(follower, new Line(
                                shootA,
                                intakeBEnd
                        )).setMaxSpeed(900).race(
                                new IntakeUntilFull(intake)
                        ),

                        new Follow(follower, new Line(
                                intakeBEnd,
                                shootB
                        )),

                        new Shoot(intake, shooter),

                        // GATE CYCLE

                        new Series(
                                new Follow(follower, new Line(
                                        shootB,
                                        gateIntake
                                )).setMaxSpeed(900),

                                new IntakeUntilFullTimeout(intake, 2),

                                new Follow(follower, new Line(
                                        gateIntake, shootB
                                )),
                                new Shoot(intake, shooter)
                        ),

                        // GATE CYCLE

                        new Series(
                                new Follow(follower, new Line(
                                        shootB,
                                        gateIntake
                                )).setMaxSpeed(900),

                                new IntakeUntilFullTimeout(intake, 2),

                                new Follow(follower, new Line(
                                        gateIntake, shootB
                                )),
                                new Shoot(intake, shooter)
                        ),

                        // GATE CYCLE

                        new Series(
                                new Follow(follower, new Line(
                                        shootB,
                                        gateIntake
                                )).setMaxSpeed(900),

                                new IntakeUntilFullTimeout(intake, 2),

                                new Follow(follower, new Line(
                                        gateIntake, shootB
                                )),
                                new Shoot(intake, shooter)
                        ),

                        new Run(() -> Shooter.ShooterPID.useConvergence = true),

                        // PARK
                        new Follow(follower, new Line(
                                shootB,
                                endPoint
                        ))
                    )
                .registerSystem(shooter)
                .registerSystem(intake)
                .registerSystem(follower);

    }
}
