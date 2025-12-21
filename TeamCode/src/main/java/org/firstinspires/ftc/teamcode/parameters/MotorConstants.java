package org.firstinspires.ftc.teamcode.parameters;

import com.qualcomm.robotcore.hardware.DcMotor;

public class MotorConstants {
    public static class Names {
        public static String leftShooterMotor = "leftShooterMotor";
        public static String rightShooterMotor = "rightShooterMotor";
    }

    public static class Reversed {
        public static boolean leftShooterMotor = false;
        public static boolean rightShooterMotor = true;
    }

    public static class ZPB {
        public static DcMotor.ZeroPowerBehavior shooterMotors = DcMotor.ZeroPowerBehavior.FLOAT;
    }
}
