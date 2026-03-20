package org.firstinspires.ftc.teamcode.Opmodes.Debug;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.lioncore.hardware.LimelightProxy;
import org.firstinspires.ftc.teamcode.lioncore.tasks.Jobs;
import org.firstinspires.ftc.teamcode.lioncore.tasks.TaskOpMode;
import org.firstinspires.ftc.teamcode.systems.Limelight;

@TeleOp
public class LimelightDebug extends TaskOpMode {

    LimelightProxy proxy;

    public Jobs spawn() {

        Limelight limelight = new Limelight();

        return Jobs.create()
                .registerSystem(limelight);
    }
}
