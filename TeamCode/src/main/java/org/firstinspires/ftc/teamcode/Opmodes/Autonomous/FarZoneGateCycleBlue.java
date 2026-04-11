package org.firstinspires.ftc.teamcode.Opmodes.Autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.lioncore.math.types.Position;
import org.firstinspires.ftc.teamcode.lioncore.paths.Line;
import org.firstinspires.ftc.teamcode.lioncore.tasks.Jobs;
import org.firstinspires.ftc.teamcode.lioncore.tasks.Sleep;
import org.firstinspires.ftc.teamcode.lioncore.tasks.TaskOpMode;
import org.firstinspires.ftc.teamcode.lioncore.tasks.WaitUntil;
import org.firstinspires.ftc.teamcode.systems.Follower;
import org.firstinspires.ftc.teamcode.systems.Intake;
import org.firstinspires.ftc.teamcode.systems.Shooter;
import org.firstinspires.ftc.teamcode.tasks.Follow;
import org.firstinspires.ftc.teamcode.tasks.IntakeUntilFull;
import org.firstinspires.ftc.teamcode.tasks.Shoot;
import org.firstinspires.ftc.teamcode.tasks.Shoot;

@Autonomous
public class FarZoneGateCycleBlue extends TaskOpMode {
    @Override
    public Jobs spawn() {

        double xOffset = 0;
        double yOffset = -150;

        Follower follower = new Follower(3200 + xOffset, 1200 + yOffset, 180);
        Intake intake = new Intake();
        intake.loadHardware(hardwareMap);
        Shooter shooter = new Shooter(intake.yieldTurretEncoder());

        Position start = new Position(3200 + xOffset, 1200 + yOffset, 180);
        Position shoot = new Position(3000 + xOffset, 1300 + yOffset, 180);
        Position wallIntakeA = new Position(3200 + xOffset, 10 + yOffset, 180);
        Position wallIntakeB = new Position(3100 + xOffset, 100 + yOffset, 180);
        Position wallIntakeC = new Position(3100 + xOffset, 100 + yOffset, 180);
        Position intakeAStart = new Position(2500 + xOffset, 1000 + yOffset, 180);
        Position intakeAEnd = new Position(2500 + xOffset, 300 + yOffset, 180);
        Position intakeBStart = new Position(1900 + xOffset, 900 + yOffset, 180);
        Position intakeBEnd = new Position(1900 + xOffset, 300 + yOffset, 180);
        Position gateA = new Position(1750 + xOffset, 600 + yOffset, 180);
        Position gateB = new Position(1750 + xOffset, 100 + yOffset, 180);

        return Jobs.create()
                .addSeries(
                        new WaitUntil(shooter::atSpeed).with(
                                new Follow(follower, new Line(
                                        start,
                                        shoot
                                )).setMaxSpeed(900)
                        ),
                        new Sleep(0.5),
                        new Shoot(intake, shooter),

                        new Follow(follower, new Line(
                                shoot, wallIntakeA
                        )).setMaxSpeed(1000).then(
                                new Sleep(0.5).then(
                                        new Follow(follower, new Line(
                                            wallIntakeA,
                                            shoot
                                        )
                                ).setMaxSpeed(600))
                        ).race(
                                new IntakeUntilFull(intake)
                        ),
                        new Sleep(0.5),
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
                        new Sleep(0.5),
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
                                gateA
                        )),
                        new Follow(follower, new Line(
                                gateA,
                                gateB
                        )),

                        new Follow(follower, new Line(
                                gateB,
                                shoot
                        )).setMaxSpeed(900),
                        new Sleep(0.5),
                        new Shoot(intake, shooter),

                        new Follow(follower, new Line(
                                shoot, wallIntakeC
                        )).race(
                                new IntakeUntilFull(intake)
                        ),
                        new Follow(follower, new Line(
                                wallIntakeC,
                                shoot
                        )),
                        new Sleep(0.5),
                        new Shoot(intake, shooter),

                        new Follow(follower, new Line(
                                shoot, wallIntakeB
                        )).race(
                                new IntakeUntilFull(intake)
                        ),
                        new Follow(follower, new Line(
                                wallIntakeB,
                                shoot
                        )),
                        new Sleep(0.5),
                        new Shoot(intake, shooter)
                )
                .registerSystem(shooter)
                .registerSystem(intake)
                .registerSystem(follower);

    }
}
