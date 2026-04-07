package org.firstinspires.ftc.teamcode.Opmodes.Autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.lioncore.math.types.Position;
import org.firstinspires.ftc.teamcode.lioncore.paths.Line;
import org.firstinspires.ftc.teamcode.lioncore.tasks.Jobs;
import org.firstinspires.ftc.teamcode.lioncore.tasks.Repeat;
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

        Follower follower = new Follower(300, 800, 180);
        Intake intake = new Intake();
        intake.loadHardware(hardwareMap);
        Shooter shooter = new Shooter(intake.yieldTurretEncoder());

        Position start = new Position(300, 800, 180);
        Position shootA = new Position(1600, 1400, 180);
        Position shootB = new Position(1600, 1400, 145);
        Position intakeAEnd = new Position(2300, 400, 180);
        Position gateIntake = new Position(2200, 250, 145);

        return Jobs.create()
                .addSeries(
                        new Follow(follower, new Line(
                                start,
                                shootA
                        )).setMaxSpeed(800).with(
                                new WaitUntil(shooter::atSpeed).then(
                                        new Sleep(1).then(
                                                new Shoot(intake, shooter)
                                        )
                                )
                        ),
                        new Follow(follower, new Line(
                                shootA, intakeAEnd
                        )).setMaxSpeed(800).race(
                                new IntakeUntilFull(intake)
                        ),
                        new Follow(follower, new Line(
                                intakeAEnd,
                                shootA
                        )).with(new Sleep(1).then(
                                new Shoot(intake, shooter)
                        )),

                        new Repeat(
                                new Series(
                                    new Follow(follower, new Line(
                                        shootB,
                                        gateIntake
                                    )).setMaxSpeed(900),

                                    new IntakeUntilFullTimeout(intake, 2),

                                    new Follow(follower, new Line(
                                        gateIntake, shootB
                                    )).setMaxSpeed(900).with(new Sleep(1).then(
                                            new Shoot(intake, shooter))
                                    )
                                ),
                            4
                        )
                    )
                .registerSystem(shooter)
                .registerSystem(intake)
                .registerSystem(follower);

    }
}
