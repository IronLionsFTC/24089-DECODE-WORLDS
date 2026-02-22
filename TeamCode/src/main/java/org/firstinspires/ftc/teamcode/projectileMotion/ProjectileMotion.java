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

    private static final Vector3 target = Vector3.zero();

    public ProjectileMotion(double velocity, double angle, double yaw) {
        this.launchVelocity = velocity;
        this.launchAngle = angle;
        this.yawDirection = yaw;
    }

    @Config
    public static class ShootOnTheMoveConstants {
        public static double headingTimeStep = 0.2;
    }

    public static ProjectileMotion calculate(Vector3 target, double currentVelocity) {
        currentVelocity *= Shooter.ShooterPID.underShoot;
        // ── Shooter position in field coordinates ────────────────────────────────
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

        // ── Target in shooter-relative ballistic coordinates ─────────────────────
        Vector3 relativeTarget = target.subtract(shooterPositionInField);
        double x = Math.hypot(relativeTarget.getX(), relativeTarget.getY()); // horizontal distance
        double y = relativeTarget.getZ();                                      // vertical distance (+ = above shooter)
        Vector2 groundPlane = Vector2.cartesian(relativeTarget.getX(), relativeTarget.getY());

        // ── Velocity required at the preferred hood angle ────────────────────────
        // v = x * sqrt( g / (2*cos²θ*(x*tanθ - y)) )
        double idealAngle  = Math.toRadians(Shooter.ShooterPID.preferredHoodAngle);
        double velocityForIdealLaunch = requiredLaunchVelocity(Vector2.cartesian(x, y), idealAngle);

        // ── Hood angle required at the current (actual) velocity ─────────────────
        // Rearranging y = x*tan(θ) - (g*x²/2v²)*(1+tan²θ) gives a quadratic in tan(θ):
        //   (g*x²/2v²)*tan²θ - x*tan(θ) + (y + g*x²/2v²) = 0
        // Let k = g*x²/(2*v²), then:  k*u² - x*u + (y+k) = 0
        // Use the + root for the high arc (above 45°), - root for low arc (below 45°)
        double v           = currentVelocity;
        double k           = G * x * x / (2 * v * v);
        double disc        = x * x - 4 * k * (y + k);
        double hoodAngle;
        if (disc < 0) {
            // Current velocity can't reach the target at any angle — fall back to ideal
            hoodAngle = idealAngle;
        } else {
            double sqrtDisc   = Math.sqrt(disc);
            double highArcTan = (x + sqrtDisc) / (2 * k); // + root → steeper (high arc)
            double lowArcTan  = (x - sqrtDisc) / (2 * k); // - root → shallower (low arc)
            // Pick whichever arc angle is closest to the preferred hood angle
            double highArc    = Math.atan(highArcTan);
            double lowArc     = Math.atan(lowArcTan);

            hoodAngle = lowArc;
        }

        // ── Turret direction with heading feedforward ────────────────────────────
        double direction = 180 + groundPlane.polarDirection()
                + SwerveDrive.PinpointCache.position.heading
                + SwerveDrive.PinpointCache.angularVelocity * ShootOnTheMoveConstants.headingTimeStep;
        while (direction < -180) direction += 360;
        while (direction >  180) direction -= 360;

        return new ProjectileMotion(velocityForIdealLaunch / Shooter.ShooterPID.underShoot, Math.toDegrees(hoodAngle), direction);
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

    public static double requiredLaunchVelocity(Vector2 target, double launchAngle) {

        double x = target.x();
        double y = target.y();

        double theta = launchAngle;
        double g = -G; // e.g. -9800 (mm/s^2)

        double cos = Math.cos(theta);
        double tan = Math.tan(theta);

        // Prevent division by zero (vertical shot)
        if (Math.abs(cos) < 1e-9) {
            throw new IllegalArgumentException("Launch angle too close to vertical.");
        }

        double denominator = 2.0 * cos * cos * (y - x * tan);

        // If denominator == 0 or sign makes v^2 negative → no solution
        double vSquared = (g * x * x) / denominator;

        if (vSquared <= 0 || Double.isNaN(vSquared) || Double.isInfinite(vSquared)) {
            throw new IllegalArgumentException("No physical solution at this launch angle.");
        }

        return Math.sqrt(vSquared);
    }

}
