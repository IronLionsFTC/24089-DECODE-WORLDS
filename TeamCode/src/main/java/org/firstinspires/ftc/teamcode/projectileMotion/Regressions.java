package org.firstinspires.ftc.teamcode.projectileMotion;

public class Regressions {
    public static class Velocity {
        public static double a = 1.69502;
    }

    public static class Angle {
        public static double a = 1.30326;
        public static double c = -17.15683;
    }

    public static double rpmToVelocity(double rpm) {
        return rpm * Velocity.a;
    }

    public static double velocityToRpm(double velocity) {
        return velocity / Velocity.a;
    }

    public static double hoodAngleToLaunchAngle(double hoodAngle) {
        return hoodAngle * Angle.a + Angle.c;
    }

    public static double launchAngleToHoodAngle(double launchAngle) {
        return launchAngle * 1.2;//return (launchAngle - Angle.c) / Angle.a;
    }
}
