package org.firstinspires.ftc.teamcode.Opmodes.Debug;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.lioncore.tasks.Jobs;
import org.firstinspires.ftc.teamcode.lioncore.tasks.TaskOpMode;
import org.firstinspires.ftc.teamcode.systems.Shooter;

@TeleOp
public class Tuning extends TaskOpMode{

    private Shooter shooter;

    public Jobs spawn() {
        this.shooter = new Shooter();

        return Jobs.create()
                .registerSystem(shooter);
    }
}

