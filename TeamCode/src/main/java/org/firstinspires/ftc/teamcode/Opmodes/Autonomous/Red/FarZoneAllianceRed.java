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
import org.firstinspires.ftc.teamcode.systems.Shooter;
import org.firstinspires.ftc.teamcode.tasks.Follow;
import org.firstinspires.ftc.teamcode.tasks.IntakeUntilFull;
import org.firstinspires.ftc.teamcode.tasks.Shoot;

@Autonomous
public class FarZoneAllianceRed extends TaskOpMode {
    @Override
    public Jobs spawn() {

        double xOffset = 0;
        double yOffset = 150;

        Follower follower = new Follower(-3200 + xOffset, 1200 + yOffset, 180);
        Intake intake = new Intake();
        intake.loadHardware(hardwareMap);
        Shooter shooter = new Shooter(intake.yieldTurretEncoder());

        Position start = new Position(-3200 + xOffset, 1200 + yOffset, 180);
        Position shoot = new Position(-3000 + xOffset, 1300 + yOffset, 180);

        Position wallIntakeA = new Position(-3190 + xOffset, 80 + yOffset, 180);
        Position wallIntakeB = new Position(-2500 + xOffset, 120 + yOffset, 210);

        Position intakeAStart = new Position(-2500 + xOffset, 1000 + yOffset, 180);
        Position intakeAEnd = new Position(-2500 + xOffset, 300 + yOffset, 180);
        Position intakeBStart = new Position(-1900 + xOffset, 900 + yOffset, 180);
        Position intakeBEnd = new Position(-1900 + xOffset, 300 + yOffset, 180);

        return Jobs.create()
                .addSeries(
                        new Run(() -> Shooter.ShooterPID.useConvergence = false),
                        new WaitUntil(shooter::atSpeed).with(
                                new Follow(follower, new Line(
                                        start,
                                        shoot
                                )).setMaxSpeed(900)
                        ),
                        new Sleep(0.2),
                        new Shoot(intake, shooter),

                        new Follow(follower, new Line(
                                shoot, wallIntakeA
                        )).setMaxSpeed(1000).then(
                                new Sleep(0.2).then(
                                        new Follow(follower, new Line(
                                            wallIntakeA,
                                            shoot
                                        )
                                ).setMaxSpeed(600))
                        ).race(
                                new IntakeUntilFull(intake)
                        ),
                        new Sleep(0.2),
                        new Shoot(intake, shooter),
                        new Follow(follower, new Line(
                                shoot,
                                intakeAStart
                        )).then(
                                new Sleep(0.2).then(new Follow(follower, new Line(
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
                        new Sleep(0.2),
                        new Shoot(intake, shooter),
                        new Follow(follower, new Line(
                                shoot,
                                intakeBStart
                        )).then(
                                new Follow(follower, new Line(
                                        intakeBStart,
                                        intakeBEnd
                                )).setMaxSpeed(900)
                        ).race(
                                new IntakeUntilFull(intake)
                        ),

                        new Follow(follower, new Line(
                                intakeBEnd,
                                shoot
                        )).setMaxSpeed(900),
                        new Sleep(0.2),
                        new Shoot(intake, shooter),

                        // Wall cycle
                        new Follow(follower, new Line(
                                shoot, wallIntakeA
                        )).then(
                                new Follow(follower, new Line(
                                        wallIntakeA, wallIntakeB
                                ))
                        ).race(
                                new IntakeUntilFull(intake)
                        ),
                        new Sleep(0.2),
                        new Follow(follower, new Line(
                                wallIntakeB, shoot
                        )),

                        // Wall cycle
                        new Follow(follower, new Line(
                                shoot, wallIntakeA
                        )).then(
                                new Follow(follower, new Line(
                                        wallIntakeA, wallIntakeB
                                ))
                        ).race(
                                new IntakeUntilFull(intake)
                        ),
                        new Sleep(0.2),
                        new Follow(follower, new Line(
                                wallIntakeB, shoot
                        )),

                        // Wall cycle
                        new Follow(follower, new Line(
                                shoot, wallIntakeA
                        )).then(
                                new Follow(follower, new Line(
                                        wallIntakeA, wallIntakeB
                                ))
                        ).race(
                                new IntakeUntilFull(intake)
                        ),
                        new Sleep(0.2),
                        new Follow(follower, new Line(
                                wallIntakeB, shoot
                        )),

                        // Wall cycle
                        new Follow(follower, new Line(
                                shoot, wallIntakeA
                        )).then(
                                new Follow(follower, new Line(
                                        wallIntakeA, wallIntakeB
                                ))
                        ).race(
                                new IntakeUntilFull(intake)
                        ),
                        new Sleep(0.2),
                        new Follow(follower, new Line(
                                wallIntakeB, shoot
                        )),

                        // Wall cycle
                        new Follow(follower, new Line(
                                shoot, wallIntakeA
                        )).then(
                                new Follow(follower, new Line(
                                        wallIntakeA, wallIntakeB
                                ))
                        ).race(
                                new IntakeUntilFull(intake)
                        ),
                                new Sleep(0.2),
                                new Follow(follower, new Line(
                                        wallIntakeB, shoot
                                ))
                )
                .registerSystem(shooter)
                .registerSystem(intake)
                .registerSystem(follower);

    }
}
