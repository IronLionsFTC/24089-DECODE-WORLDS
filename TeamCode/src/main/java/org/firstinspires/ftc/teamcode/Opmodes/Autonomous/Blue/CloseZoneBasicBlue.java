package org.firstinspires.ftc.teamcode.Opmodes.Autonomous.Blue;

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
public class CloseZoneBasicBlue extends TaskOpMode {
    @Override
    public Jobs spawn() {

        double shootDelay = 0.2;
        double xOffset = -50;
        double yOffset = 50;

        Follower follower = new Follower(0 + xOffset, 500 + yOffset, 180);
        Intake intake = new Intake();
        intake.loadHardware(hardwareMap);
        Shooter shooter = new Shooter(intake.yieldTurretEncoder());

        Position start = new Position(0 + xOffset, 500 + yOffset, 180);
        Position firstThree = new Position(1000 + xOffset, 800 + yOffset, 180);
        Position elbow = new Position(1900 + xOffset, 800 + yOffset, 180);
        Position shootA = new Position(1500 + xOffset, 1200 + yOffset, 150);
        Position intakeAEnd = new Position(2000 + xOffset, -150 + yOffset, 180);
        Position intakeBEnd = new Position(1350 + xOffset, -60 + yOffset, 180);
        Position gate = new Position(1750 + xOffset, 20 + yOffset, 150);
        Position endPoint = new Position(2000 + xOffset, 300 + yOffset, 150);

        return Jobs.create()
                .addSeries(
                        new Run(() -> Shooter.ShooterPID.useConvergence = false),

                        new Follow(follower, new Line(
                                start,
                                firstThree
                        )).setMaxSpeed(1000),

                        new Sleep(shootDelay).then(new Shoot(intake, shooter)),

                        new Follow(follower, new Line(
                                firstThree,
                                elbow
                        )),

                        new Follow(follower, new Line(
                                elbow, intakeAEnd
                        )).setMaxSpeed(800).race(
                                new IntakeUntilFull(intake)
                        ),
                        new Follow(follower, new Line(
                                intakeAEnd,
                                shootA
                        )),

                        new Sleep(shootDelay).then(new Shoot(intake, shooter)),

                        // SECOND ROW

                        new Follow(follower, new Line(
                                shootA,
                                intakeBEnd
                        )).setMaxSpeed(800).race(
                                new IntakeUntilFull(intake)
                        ),

                        new Follow(follower, new Line(
                                intakeBEnd,
                                shootA
                        )),

                        new Sleep(shootDelay).then(new Shoot(intake, shooter)),

                        new Follow(follower, new Line(
                                shootA,
                                gate
                        )),

                        new Sleep(12),

                        new Follow(follower, new Line(
                                gate,
                                endPoint
                        ))
                    )
                .registerSystem(shooter)
                .registerSystem(intake)
                .registerSystem(follower);

    }
}
