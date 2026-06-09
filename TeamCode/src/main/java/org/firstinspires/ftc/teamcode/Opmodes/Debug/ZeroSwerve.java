package org.firstinspires.ftc.teamcode.Opmodes.Debug;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.lioncore.hardware.LionServo;
import org.firstinspires.ftc.teamcode.parameters.ServoConstants;

@TeleOp
public class ZeroSwerve extends LinearOpMode {

    private static final double STEP = 0.002;

    public void runOpMode() {

        LionServo rightFront = LionServo.single(hardwareMap, ServoConstants.Names.rightFront, 0.5);
        LionServo leftFront  = LionServo.single(hardwareMap, ServoConstants.Names.leftFront,  0.5);
        LionServo rightRear  = LionServo.single(hardwareMap, ServoConstants.Names.rightRear,  0.5);
        LionServo leftRear   = LionServo.single(hardwareMap, ServoConstants.Names.leftRear,   0.5);

        double rfPos = 0.5;
        double lfPos = 0.5;
        double rrPos = 0.5;
        double lrPos = 0.5;

        waitForStart();

        while (opModeIsActive()) {

            if (gamepad1.dpad_up)    rfPos = clamp(rfPos + STEP);
            if (gamepad1.dpad_down)  lfPos = clamp(lfPos + STEP);
            if (gamepad1.dpad_left)  rrPos = clamp(rrPos + STEP);
            if (gamepad1.dpad_right) lrPos = clamp(lrPos + STEP);

            rightFront.setPosition(rfPos);
            leftFront.setPosition(lfPos);
            rightRear.setPosition(rrPos);
            leftRear.setPosition(lrPos);

        }
    }

    private double clamp(double pos) {
        return Math.max(0.0, Math.min(1.0, pos));
    }
}