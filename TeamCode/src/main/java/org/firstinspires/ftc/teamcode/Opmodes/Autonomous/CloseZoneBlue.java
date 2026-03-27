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

        Follower follower = new Follower(300, 600, 180);
        Intake intake = new Intake();
        intake.loadHardware(hardwareMap);
        Shooter shooter = new Shooter(intake.yieldTurretEncoder());
        Shooter.ShooterPID.useConvergence = false;

        Position start = new Position(300, 600, 180);
        Position shoot = new Position(1500, 1100, 150);
        Position intakeAEnd = new Position(2300, 400, 180);
        Position gateIntakeA = new Position(2100, 200, 150);
        Position gateIntakeB = new Position(2400, 100, 135);
        Position intakeBEnd = new Position(1400, 300, 180);

        return Jobs.create()
                .addSeries(
                        new WaitUntil(shooter::atSpeed).with(
                                new Follow(follower, new Line(
                                        start,
                                        shoot
                                ))
                        ),
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
                        )).then(
                                new Follow(follower, new Line(
                                        gateIntakeA,
                                        gateIntakeB
                                )),
                                new Sleep(2)
                        ).race(
                                new IntakeUntilFull(intake)
                        )
                )
                .registerSystem(shooter)
                .registerSystem(intake)
                .registerSystem(follower);

    }
}
