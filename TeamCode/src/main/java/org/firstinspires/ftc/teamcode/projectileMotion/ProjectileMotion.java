package org.firstinspires.ftc.teamcode.projectileMotion;

import com.acmerobotics.dashboard.config.Config;

import org.firstinspires.ftc.teamcode.lioncore.math.types.Vector2;
import org.firstinspires.ftc.teamcode.lioncore.math.types.Vector3;
import org.firstinspires.ftc.teamcode.parameters.Zeroing;
import org.firstinspires.ftc.teamcode.systems.Shooter;
import org.firstinspires.ftc.teamcode.systems.SwerveDrive;

public class ProjectileMotion {

    public static double G = 9800;
    public double launchVelocity;
    public double launchAngle;
    public double yawDirection;
    public boolean possible;

    private static final Vector3 target = Vector3.zero();

    public ProjectileMotion(double velocity, double angle, double yaw, boolean possible) {
        this.launchVelocity = velocity;
        this.launchAngle = angle;
        this.yawDirection = yaw;
        this.possible = possible;
    }

    @Config
    public static class ShootOnTheMoveConstants {
        public static double headingTimeStep = 0.2;
    }

    public static ProjectileMotion calculate(Vector3 target, double currentVelocity) {
        currentVelocity *= Shooter.ShooterPID.underShoot;
        double cosHeading = Math.cos(SwerveDrive.PinpointCache.position.heading);
        double sinHeading = Math.sin(SwerveDrive.PinpointCache.position.heading);
        Vector3 shooterPositionInField = new Vector3(
                SwerveDrive.PinpointCache.position.position.x()
                        + Zeroing.ProjMotConstants.shooterOffset.getX() * cosHeading
                        - Zeroing.ProjMotConstants.shooterOffset.getY() * sinHeading,
                SwerveDrive.PinpointCache.position.position.y()
                        + Zeroing.ProjMotConstants.shooterOffset.getX() * sinHeading
                        + Zeroing.ProjMotConstants.shooterOffset.getY() * cosHeading,
                Zeroing.ProjMotConstants.shooterOffset.getZ()
        );

        Vector3 relativeTarget = target.subtract(shooterPositionInField);
        double x = Math.hypot(relativeTarget.getX(), relativeTarget.getY()); // horizontal distance
        double y = relativeTarget.getZ();                                      // vertical distance (+ = above shooter)
        Vector2 groundPlane = Vector2.cartesian(relativeTarget.getX(), relativeTarget.getY());

        // ── Turret direction with heading feedforward ────────────────────────────
        double direction = 180 + groundPlane.polarDirection()
                + SwerveDrive.PinpointCache.position.heading
                + SwerveDrive.PinpointCache.angularVelocity * ShootOnTheMoveConstants.headingTimeStep;
        while (direction < -180) direction += 360;
        while (direction >  180) direction -= 360;


        // Projectile math
        double chunk = (G * Math.pow(x, 2)) / (2 * Math.pow(currentVelocity, 2));
        double dscrm = Math.pow(x, 2) - 4 * chunk * (chunk + y);

        if (dscrm < 0) {
            return new ProjectileMotion(4000, 45, direction, false);
        }

        double plus = (x + Math.sqrt(dscrm)) / (chunk * 2);
        double minus = (x - Math.sqrt(dscrm)) / (chunk * 2);

        double angle = Math.min(Math.toDegrees(Math.atan(plus)), Math.toDegrees(Math.atan(minus)));

        return new ProjectileMotion(Shooter.ShooterPID.velocityOverride / Shooter.ShooterPID.underShoot, angle, direction, true);
    }

    public static boolean far() {
        double distance = SwerveDrive.PinpointCache.position.position.sub( Vector2.cartesian(Shooter.ShooterPID.targetXFar, Shooter.ShooterPID.targetYFar) ).magnitude();
        return distance > 2500;
    }

    public static Vector3 getTarget() {
        if (far()) {
            target.setX(Shooter.ShooterPID.targetXFar);
            target.setY(Shooter.ShooterPID.targetYFar);
            target.setZ(Shooter.ShooterPID.targetZFar);
        } else {
            target.setX(Shooter.ShooterPID.targetXClose);
            target.setY(Shooter.ShooterPID.targetYClose);
            target.setZ(Shooter.ShooterPID.targetZClose);
        } return target;
    }

}
