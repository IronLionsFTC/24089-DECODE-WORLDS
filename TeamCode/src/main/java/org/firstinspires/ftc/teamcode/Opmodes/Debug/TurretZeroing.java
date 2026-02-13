package org.firstinspires.ftc.teamcode.Opmodes.Debug;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.lioncore.hardware.LionServo;

@TeleOp
public class TurretZeroing extends OpMode {

    private LionServo turretServo;
    private LionServo blockerServo;

    @Config
    public static class TurretZeroPosition {
        public static double turretZeroPosition = 0.5;
        public static double blockerZeroPosition = 0.0;
    }

    public void init() {
        turretServo = LionServo.mirrored(hardwareMap, "leftTurretServo", "rightTurretServo", 0.5);
        blockerServo = LionServo.single(hardwareMap, "blocker", 0.0);
    }

    public void loop() {
        turretServo.setPosition(TurretZeroPosition.turretZeroPosition);
        blockerServo.setPosition(TurretZeroPosition.blockerZeroPosition);
    }

}
