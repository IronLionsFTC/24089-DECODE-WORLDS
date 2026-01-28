package org.firstinspires.ftc.teamcode.Opmodes.Debug;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.lioncore.math.types.Position;
import org.firstinspires.ftc.teamcode.lioncore.tasks.Jobs;
import org.firstinspires.ftc.teamcode.lioncore.tasks.TaskOpMode;
import org.firstinspires.ftc.teamcode.systems.Follower;
import org.firstinspires.ftc.teamcode.systems.SwerveDrive;

@Autonomous
public class LineTest extends TaskOpMode {

    @Override
    public Jobs spawn() {

        SwerveDrive swerveDrive = new SwerveDrive(new Position(0, 0, 0));
        Follower follower = new Follower(swerveDrive);

        return Jobs.create()
                .registerSystem(swerveDrive)
                .registerSystem(follower);
    }
}
