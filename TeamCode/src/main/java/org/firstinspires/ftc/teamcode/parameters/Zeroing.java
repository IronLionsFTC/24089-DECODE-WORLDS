package org.firstinspires.ftc.teamcode.parameters;

public class Zeroing {
    public static class Names {
        public static String rightFrontAnalog = "rightFrontAnalog";
        public static String leftFrontAnalog = "leftFrontAnalog";
        public static String rightRearAnalog = "rightRearAnalog";
        public static String leftRearAnalog = "leftRearAnalog";
    }

    public static class ZeroPositions {
        public static double rightFront = 1.537;
        public static double leftFront = 3.155;
        public static double rightRear = 2.844;
        public static double leftRear = 2.492;
    }

    public static class Constants {
        public static double voltageRange = 3.3;
        public static double scale = 360 / voltageRange;
    }

    public static double podAngle(double voltage, double zeroPosition) {
        double unwrapped = (voltage - zeroPosition) * Zeroing.Constants.scale;
        while (unwrapped < -180) unwrapped += 360;
        while (unwrapped > 180) unwrapped -= 360;
        return unwrapped;
    }

    public static double polarQuadrature(double rawTicks) {
        double degrees = rawTicks * (360.0 / 4096.0);
        while (degrees < -180) degrees += 360;
        while (degrees > 180) degrees += 360;
        return degrees;
    }
}
