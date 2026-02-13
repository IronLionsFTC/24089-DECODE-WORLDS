package org.firstinspires.ftc.teamcode.parameters;

import com.qualcomm.robotcore.hardware.DcMotor;

public class MotorConstants {
    public static class Names {
        public static String rightFront = "rightFront";
        public static String leftFront = "leftFront";
        public static String rightRear = "rightRear";
        public static String leftRear = "leftRear";
        public static String leftShooterMotor = "leftShooterMotor";
        public static String rightShooterMotor = "rightShooterMotor";
        public static String intakeMotor = "intakeMotor";
        public static String transferMotor = "transferMotor";
    }

    public static class Reversed {
        public static boolean leftShooterMotor = true;
        public static boolean rightShooterMotor = false;
        public static boolean intakeMotor = false;
        public static boolean transferMotor = true;
        public static boolean rf = true;
        public static boolean lf = false;
        public static boolean rr = true;
        public static boolean lr = false;
    }

    public static class ZPB {
        public static DcMotor.ZeroPowerBehavior shooterMotors = DcMotor.ZeroPowerBehavior.FLOAT;
        public static DcMotor.ZeroPowerBehavior driveMotors = DcMotor.ZeroPowerBehavior.FLOAT;
        public static DcMotor.ZeroPowerBehavior intakeMotors = DcMotor.ZeroPowerBehavior.BRAKE;
    }
}
