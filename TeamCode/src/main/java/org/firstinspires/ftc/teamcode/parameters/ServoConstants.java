package org.firstinspires.ftc.teamcode.parameters;

public class ServoConstants {
    public static class Names {
        public static String hoodServo = "hoodServo";
        public static String rightFront = "rightFrontServo";
        public static String leftFront = "leftFrontServo";
        public static String rightRear = "rightRearServo";
        public static String leftRear = "leftRearServo";

        public static String leftTurretServo = "leftTurretServo";
        public static String rightTurretServo = "rightTurretServo";

        public static String blocker = "blocker";
    }

    public static class Zero {
        public static double hood = 0;
        public static double turret = 0.0;
    }

    public static class Positions {
        public static double blockerOpen = 0.0;
        public static double blockerClosed = 0.25;
    }

    public static class Ratios {
        public static double hoodRatio = (40.0 / 15.0) * (15.0 / 164.0);
        public static double hoodAngle = 255;
        public static double hoodZeroAngle = 23;

        public static double turret = 24.0 / 86.0;
    }
}
