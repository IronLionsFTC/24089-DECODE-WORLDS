package org.firstinspires.ftc.teamcode.parameters;

import org.firstinspires.ftc.teamcode.lioncore.math.types.Vector3;

public class Zeroing {
    public static class Names {
        public static String rightFrontAnalog = "rightFrontAnalog";
        public static String leftFrontAnalog = "leftFrontAnalog";
        public static String rightRearAnalog = "rightRearAnalog";
        public static String leftRearAnalog = "leftRearAnalog";
    }

    public static class Constants {
        public static double voltageRange = 3.3;
        public static double scale = 360 / voltageRange;
    }

    public static double podAngle(double voltage, double zeroPosition) {
        double unwrapped = (zeroPosition - voltage) * Zeroing.Constants.scale;
        while (unwrapped < -180) unwrapped += 360;
        while (unwrapped > 180) unwrapped -= 360;
        return unwrapped;
    }

    public static double polarQuadrature(double rawTicks) {
        double degrees = rawTicks * (360.0 / 4096.0);
        while (degrees < -180) degrees += 360;
        while (degrees > 180) degrees -= 360;
        return degrees;
    }

    public static class ProjMotConstants {
        public static Vector3 shooterOffset = new Vector3(0, -20, 300);
    }

}
