package org.firstinspires.ftc.teamcode.projectileMotion;

import com.acmerobotics.dashboard.config.Config;

public class Regressions {
    public static class Velocity {
        public static double a = -0.000191849;
        public static double b = 2.57433;
    }

    @Config
    public static class Angle {
        public static double a = 1.16209;
        public static double c = -5.64129;
    }

    public static double rpmToVelocity(double rpm) {
        return rpm * rpm * Velocity.a + rpm * Velocity.b;
    }

    public static double velocityToRpm(double velocity) {
        return (-Velocity.b + Math.sqrt(Velocity.b * Velocity.b + 4 * Velocity.a * velocity)) / (2 * Velocity.a);
    }

    public static double launchAngleToHoodAngle(double launchAngle) {
        return (launchAngle - Angle.c) / Angle.a;
    }
}
