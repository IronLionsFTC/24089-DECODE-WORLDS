package org.firstinspires.ftc.teamcode.Opmodes.Debug;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.lioncore.math.types.Position;
import org.firstinspires.ftc.teamcode.lioncore.paths.Line;
import org.firstinspires.ftc.teamcode.lioncore.tasks.Jobs;
import org.firstinspires.ftc.teamcode.lioncore.tasks.TaskOpMode;
import org.firstinspires.ftc.teamcode.systems.Follower;
import org.firstinspires.ftc.teamcode.tasks.Follow;

@Autonomous
public class LineTest extends TaskOpMode {

    @Override
    public Jobs spawn() {

        Follower follower = new Follower(0, 0, 90);

        return Jobs.create()
                .addSeries(
                        new Follow(follower, new Line(
                            new Position(0, 0, 90),
                            new Position(0, 1500, 60)
                        )).setMaxSpeed(1200),

                        new Follow(follower, new Line(
                                new Position(0, 0, 90),
                                new Position(0, 1500, 90)
                        )).setMaxSpeed(1200)
                )
                .registerSystem(follower);
    }
}
