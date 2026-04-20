package org.firstinspires.ftc.teamcode.Opmodes.Autonomous.Blue;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.lioncore.math.types.Position;
import org.firstinspires.ftc.teamcode.lioncore.paths.Line;
import org.firstinspires.ftc.teamcode.lioncore.tasks.Jobs;
import org.firstinspires.ftc.teamcode.lioncore.tasks.Run;
import org.firstinspires.ftc.teamcode.lioncore.tasks.Series;
import org.firstinspires.ftc.teamcode.lioncore.tasks.Sleep;
import org.firstinspires.ftc.teamcode.lioncore.tasks.TaskOpMode;
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

        double shootDelay = 0.4;
        double xOffset = -100;
        double yOffset = 150;

        Follower follower = new Follower(0 + xOffset, 500 + yOffset, 180);
        Intake intake = new Intake();
        intake.loadHardware(hardwareMap);
        Shooter shooter = new Shooter(intake.yieldTurretEncoder());

        Position start = new Position(0 + xOffset, 500 + yOffset, 180);
        Position firstThree = new Position(1000 + xOffset, 800 + yOffset, 180);
        Position elbow = new Position(1900 + xOffset, 800 + yOffset, 180);
        Position shootA = new Position(1500 + xOffset, 1200 + yOffset, 150);
        Position shootB = new Position(1300 + xOffset, 1000 + yOffset, 150);
        Position intakeAEnd = new Position(2000 + xOffset, -150 + yOffset, 180);
        Position intakeBEnd = new Position(1350 + xOffset, -60 + yOffset, 180);
        Position gateIntake = new Position(1960 + xOffset, -150 + yOffset, 150);
        Position gateIntakeB = new Position(2020 + xOffset, -150 + yOffset, 150);
        Position endPoint = new Position(2000 + xOffset, 300 + yOffset, 150);

        return Jobs.create()
                .addSeries(
                        new Run(() -> Shooter.ShooterPID.useConvergence = false),
                        new Follow(follower, new Line(
                                start,
                                firstThree
                        )).setMaxSpeed(800),

                        new Sleep(shootDelay).then(new Shoot(intake, shooter)),

                        new Follow(follower, new Line(
                                firstThree,
                                elbow
                        )),

                        new Follow(follower, new Line(
                                elbow, intakeAEnd
                        )).setMaxSpeed(900).race(
                                new IntakeUntilFull(intake)
                        ),
                        new Follow(follower, new Line(
                                intakeAEnd,
                                shootA
                        )),

                        new Sleep(shootDelay).then(new Shoot(intake, shooter)),

                        // GATE CYCLE

                        new Series(
                            new Follow(follower, new Line(
                                shootA,
                                gateIntake
                            )).setMaxSpeed(900),

                            new IntakeUntilFullTimeout(intake, 2).with(
                                    new Follow(follower, new Line(
                                            gateIntake, gateIntakeB
                                    ))
                            ),

                            new Follow(follower, new Line(
                                gateIntakeB, shootA
                            )),
                            new Sleep(shootDelay).then(new Shoot(intake, shooter))
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

                        new Sleep(shootDelay).then(new Shoot(intake, shooter)),

                        // GATE CYCLE

                        new Series(
                                new Follow(follower, new Line(
                                        shootB,
                                        gateIntake
                                )).setMaxSpeed(900),

                                new IntakeUntilFullTimeout(intake, 2).with(
                                        new Follow(follower, new Line(
                                                gateIntake, gateIntakeB
                                        ))
                                ),

                                new Follow(follower, new Line(
                                        gateIntakeB, shootB
                                )),
                                new Sleep(shootDelay).then(new Shoot(intake, shooter))
                        ),

                        // GATE CYCLE

                        new Series(
                                new Follow(follower, new Line(
                                        shootB,
                                        gateIntake
                                )).setMaxSpeed(900),

                                new IntakeUntilFullTimeout(intake, 2).with(
                                        new Follow(follower, new Line(
                                                gateIntake, gateIntakeB
                                        ))
                                ),

                                new Follow(follower, new Line(
                                        gateIntakeB, shootB
                                )),
                                new Sleep(shootDelay).then(new Shoot(intake, shooter))
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
