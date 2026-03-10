package org.firstinspires.ftc.teamcode.Opmodes.Debug;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.lioncore.math.types.Position;
import org.firstinspires.ftc.teamcode.lioncore.tasks.Forever;
import org.firstinspires.ftc.teamcode.lioncore.tasks.Jobs;
import org.firstinspires.ftc.teamcode.lioncore.tasks.TaskOpMode;
import org.firstinspires.ftc.teamcode.systems.VelocityFollower;

@TeleOp
public class FieldCentricHoldpointTest extends TaskOpMode {

    @Config
    public static class HoldpointTuning {
        public static double x = 0;
        public static double y = 0;
        public static double h = 0;
    }

    @Override
    public Jobs spawn() {
        VelocityFollower follower = new VelocityFollower(0, 0, 0);
        follower.state = VelocityFollower.State.Holdpoint;

        return Jobs.create()
                .addTask(new Forever(
                        () -> follower.setHoldpoint(new Position(HoldpointTuning.x, HoldpointTuning.y, HoldpointTuning.h))
                ))
                .registerSystem(follower);
    }
}
