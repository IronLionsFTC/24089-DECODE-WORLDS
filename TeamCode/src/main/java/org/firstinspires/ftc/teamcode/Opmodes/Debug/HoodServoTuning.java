package org.firstinspires.ftc.teamcode.Opmodes.Debug;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.lioncore.hardware.LionServo;
import org.firstinspires.ftc.teamcode.parameters.ServoConstants;

@TeleOp
public class HoodServoTuning extends OpMode {
    private LionServo hoodServo;

    @Config
    public static class HoodServoPosition {
        public static double hoodServo = 0;
    }

    @Override
    public void init() {
        this.hoodServo = LionServo.single(hardwareMap, ServoConstants.Names.hoodServo, 0);
    }

    @Override
    public void loop() {
       this.hoodServo.setPosition(HoodServoPosition.hoodServo);
    }
}
