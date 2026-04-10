package org.firstinspires.ftc.teamcode.projectileMotion;

import com.acmerobotics.dashboard.config.Config;

public class Regressions {

    @Config
    public static class Velocity {
        public static double a = -11380.7834;
        public static double b = -0.000247539;
        public static double c = 11431.9253;
    }

    @Config
    public static class Angle {
        public static double a = 1;
        public static double cFar = 0;
        public static double cClose = -1;
    }

    public static double rpmToVelocity(double rpm) {
        return Velocity.a * Math.exp(Velocity.b * rpm) + Velocity.c;
    }

    public static double velocityToRpm(double velocity) {
        return Math.log((velocity - Velocity.c) / Velocity.a) / Velocity.b;
    }

    public static double launchAngleToHoodAngle(double launchAngle) {
        double c;
        if (ProjectileMotion.far()) c = Angle.cFar;
        else c = Angle.cClose;
        return (launchAngle - c) / Angle.a;
    }
}
