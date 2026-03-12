package org.firstinspires.ftc.teamcode.projectileMotion;

import com.acmerobotics.dashboard.config.Config;

public class Regressions {
    public static class Velocity {
        public static double a = -17985.5239;
        public static double b = -0.0000844472;
        public static double c = 20000;
    }

    @Config
    public static class Angle {
        public static double a = 1;
        public static double c = 0;
    }

    public static double rpmToVelocity(double rpm) {
        return Velocity.a * Math.exp(Velocity.b * rpm) + Velocity.c;
    }

    public static double velocityToRpm(double velocity) {
        return Math.log((velocity - Velocity.c) / Velocity.a) / Velocity.b;
    }

    public static double launchAngleToHoodAngle(double launchAngle) {
        return (launchAngle - Angle.c) / Angle.a;
    }
}
