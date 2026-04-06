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

@Autonomous
public class CloseZoneBlue extends TaskOpMode {
    @Override
    public Jobs spawn() {

        Follower follower = new Follower(300, 800, 180);
        Intake intake = new Intake();
        intake.loadHardware(hardwareMap);
        Shooter shooter = new Shooter(intake.yieldTurretEncoder());

        Position start = new Position(300, 800, 180);
        Position shoot = new Position(1600, 1400, 180);
        Position intakeAEnd = new Position(2300, 400, 210);
        Position gateIntakeA = new Position(2150, 300, 145);
        Position gateIntakeB = new Position(2400, 200, 145);
        Position intakeBEnd = new Position(1400, 700, 180);

        return Jobs.create()
                .addSeries(
                        new Follow(follower, new Line(
                                start,
                                shoot
                        )),
                        new Sleep(1),
                        new Shoot(intake, shooter),
                        new Follow(follower, new Line(
                                shoot, intakeAEnd
                        )).race(
                                new IntakeUntilFull(intake)
                        ),
                        new Follow(follower, new Line(
                                intakeAEnd,
                                shoot
                        )),
                        new Sleep(1),
                        new Shoot(intake, shooter),

                        new Follow(follower, new Line(
                                shoot,
                                gateIntakeA
                        )).then(new Sleep(0.5)).then(
                                new Follow(follower, new Line(
                                        gateIntakeA,
                                        gateIntakeB
                                )),
                                new Sleep(2)
                        ).race(
                                new IntakeUntilFull(intake)
                        ),
                        new Follow(follower, new Line(
                                gateIntakeB, shoot
                        )),
                        new Sleep(1),
                        new Shoot(intake, shooter),

                        new Follow(follower, new Line(
                                shoot,
                                gateIntakeA
                        )).then(new Sleep(0.5)).then(
                                new Follow(follower, new Line(
                                        gateIntakeA,
                                        gateIntakeB
                                )),
                                new Sleep(2)
                        ).race(
                                new IntakeUntilFull(intake)
                        ),
                        new Follow(follower, new Line(
                                gateIntakeB, shoot
                        )),
                        new Sleep(1),
                        new Shoot(intake, shooter),

                        new Follow(follower, new Line(
                                shoot,
                                gateIntakeA
                        )).then(new Sleep(0.5)).then(
                                new Follow(follower, new Line(
                                        gateIntakeA,
                                        gateIntakeB
                                )),
                                new Sleep(2)
                        ).race(
                                new IntakeUntilFull(intake)
                        ),
                        new Follow(follower, new Line(
                                gateIntakeB, shoot
                        )),
                        new Sleep(1),
                        new Shoot(intake, shooter),

                        new Follow(follower, new Line(
                                shoot,
                                gateIntakeA
                        )).then(new Sleep(0.5)).then(
                                new Follow(follower, new Line(
                                        gateIntakeA,
                                        gateIntakeB
                                )),
                                new Sleep(2)
                        ).race(
                                new IntakeUntilFull(intake)
                        ),
                        new Follow(follower, new Line(
                                gateIntakeB, shoot
                        )),
                        new Sleep(1),
                        new Shoot(intake, shooter),

                        new Follow(follower, new Line(
                                shoot,
                                intakeBEnd
                        )).race(
                                new IntakeUntilFull(intake)
                        ),
                        new Follow(follower, new Line(
                                intakeBEnd,
                                shoot
                        )),
                        new Sleep(1),
                        new Shoot(intake, shooter)
                    )
                .registerSystem(shooter)
                .registerSystem(intake)
                .registerSystem(follower);

    }
}
