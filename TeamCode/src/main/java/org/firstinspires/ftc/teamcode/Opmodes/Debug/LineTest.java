package org.firstinspires.ftc.teamcode.Opmodes.Debug;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.lioncore.math.types.Position;
import org.firstinspires.ftc.teamcode.lioncore.paths.Line;
import org.firstinspires.ftc.teamcode.lioncore.tasks.Jobs;
import org.firstinspires.ftc.teamcode.lioncore.tasks.TaskOpMode;
import org.firstinspires.ftc.teamcode.systems.Follower;
import org.firstinspires.ftc.teamcode.systems.SwerveDrive;
import org.firstinspires.ftc.teamcode.tasks.Follow;

@Autonomous
public class LineTest extends TaskOpMode {

    @Override
    public Jobs spawn() {

        Follower follower = new Follower();

        return Jobs.create()
                .addSeries(
                        new Follow(follower, new Line(
                            new Position(0, 0, 0),
                            new Position(400, 2000, 90)
                        ), false),
                        new Follow(follower, new Line(
                                new Position(400, 2000, 90),
                                new Position(800, 1600, 180)
                        ), false),
                        new Follow(follower, new Line(
                                new Position(800, 1600, 180),
                                new Position(0, 0, 0)
                        ), false)
                )
                .registerSystem(follower);
    }
}
