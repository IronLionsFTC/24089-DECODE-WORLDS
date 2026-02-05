package org.firstinspires.ftc.teamcode.Opmodes.Debug;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.lioncore.math.types.Vector;
import org.firstinspires.ftc.teamcode.lioncore.tasks.Forever;
import org.firstinspires.ftc.teamcode.lioncore.tasks.Jobs;
import org.firstinspires.ftc.teamcode.lioncore.tasks.TaskOpMode;
import org.firstinspires.ftc.teamcode.systems.VelocityFollower;

@TeleOp
public class FieldCentricVelocityTest extends TaskOpMode {

    @Config
    public static class VelocityTuning {
        public static double xVelocity = 0;
        public static double yVelocity = 0;
    }

    @Override
    public Jobs spawn() {
        VelocityFollower follower = new VelocityFollower();
        return Jobs.create()
                .addTask(new Forever(
                        () -> follower.setTargetFieldCentricVelocity(Vector.cartesian(VelocityTuning.xVelocity, VelocityTuning.yVelocity))
                ))
                .registerSystem(follower);
    }
}
