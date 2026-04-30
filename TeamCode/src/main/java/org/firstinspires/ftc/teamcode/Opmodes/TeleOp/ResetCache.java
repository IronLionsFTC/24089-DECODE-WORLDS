package org.firstinspires.ftc.teamcode.Opmodes.TeleOp;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.systems.SwerveDrive;

@TeleOp
public class ResetCache extends LinearOpMode     {
    public void runOpMode() {
        if (isStopRequested()) return;
        waitForStart();

        SwerveDrive.PinpointCache.position = null;
        SwerveDrive.PinpointCache.velocity = null;
        SwerveDrive.PinpointCache.angularVelocity = 0;
    }
}
