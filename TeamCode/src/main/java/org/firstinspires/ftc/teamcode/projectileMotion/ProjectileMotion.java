package org.firstinspires.ftc.teamcode.projectileMotion;

import com.acmerobotics.dashboard.config.Config;

import org.firstinspires.ftc.teamcode.lioncore.math.types.Vector2;
import org.firstinspires.ftc.teamcode.lioncore.math.types.Vector3;
import org.firstinspires.ftc.teamcode.parameters.Zeroing;
import org.firstinspires.ftc.teamcode.systems.Shooter;
import org.firstinspires.ftc.teamcode.systems.SwerveDrive;

public class ProjectileMotion {

    public static double G = 9800;
    public double velocity;
    public double altitude;
    public double azimuth;
    public boolean possible;
    public double time;
    public double groundDist;

    private static final Vector3 target = Vector3.zero();

    public ProjectileMotion(double velocity, double angle, double yaw, double timeOfFlight, boolean possible, double groundDist) {
        this.velocity = velocity;
        this.altitude = angle;
        this.azimuth = yaw;
        this.time = timeOfFlight;
        this.possible = possible;
        this.groundDist = groundDist;
    }

    @Config
    public static class ShootOnTheMoveConstants {
        public static double turretLookahead = 0.1;
        public static int convergence = 3;
        public static double timeOverestimate = 1.5;
        public static double lastDistance = 1000;
        public static double calcDistance = 1000;
        public static double lastVelocity = 1000;
    }

    /**
     * Solve for the velocity required to intercept a target at a given launch_angle
     * @param desiredAngleRadians Launch angle above +x axis, radians.
     * @param tx Horizontal distance to target, mm
     * @param ty Height of target, mm
     * @return Velocity, mm/s
     */
    public static double solveVelocity(double desiredAngleRadians, double tx, double ty) {
        double cos_a = Math.cos(desiredAngleRadians);
        if (cos_a < 1e-9) return Double.NaN;
        double k = tx * Math.tan(desiredAngleRadians) - ty;
        if (k < 1e-9) return Double.NaN;
        double vSquared = (G * tx * tx) / (2 * cos_a * cos_a * k);
        if (!Double.isFinite(vSquared) || vSquared < 0.0) return Double.NaN;
        return Math.sqrt(vSquared);
    }

    /**
     * Solve for the angle required to intercept a target at a measured velocity.
     * @param measuredVelocity mm/2
     * @param tx Horizontal distance to target, mm
     * @param ty Height of target, mm
     * @return Angle, radians above +x axis
     */
    public static double solveAngle(double measuredVelocity, double tx, double ty) {
        if (measuredVelocity <= 2000) return Double.NaN;
        double chunk = (G * tx * tx) / (2 * measuredVelocity * measuredVelocity);
        double dscrm = tx * tx - 4 * chunk * (chunk + ty);
        if (dscrm < 0) return Double.NaN;
        double plus = Math.atan((tx + Math.sqrt(dscrm)) / (2 * chunk));
        double minu = Math.atan((tx - Math.sqrt(dscrm)) / (2 * chunk));

        double a = Math.toDegrees(plus);
        double b = Math.toDegrees(minu);

        if (a > 60.0 || a < 30.0) return minu;
        if (b > 60.0 || b < 30.0) return plus;

        if (Shooter.ShooterPID.useMinimum) return Math.min(plus, minu);
        else return Math.max(plus, minu);
    }

    /**
     * Find an appropriate launch velocity to use
     * @param tx Horizontal distance to target, mm
     * @param ty Height of target, mm
     * @return A suitable launch velocity that works within the bounds of the hood angle
     */
    public static double findSuitableVelocity(double tx, double ty) {
        double solution = 10000;
        double solutionWeight = 10000;

        for (double angle = 30; angle < 60.0; angle += 1.0) {
            double velocity = solveVelocity(Math.toRadians(angle), tx, ty);
            if (Double.isNaN(velocity)) continue;
            if (velocity > 9000) continue;

            double a = Math.toDegrees(solveAngle(velocity,                       tx, ty));
            double b = Math.toDegrees(solveAngle(velocity - Shooter.ShooterPID.expectedDrop, tx, ty));
            double c = Math.toDegrees(solveAngle(velocity - Shooter.ShooterPID.expectedDrop * 1.5, tx, ty));

            if (a >= 20 && b >= 20 && c >= 20 && a <= 52 && b <= 52 && c <= 52) {
                double weight = velocity / 9000.0 + Math.abs(c - a) / 20.0;
                if (weight < solutionWeight) {
                    solution = velocity;
                    solutionWeight = weight;
                }
            }
        }

        if (solution == 10000) return Double.NaN;
        return solution;
    }

    public static double alternativeFindSuitableVelocity(double tx, double ty) {
        double solution = 10000;
        double solutionWeight = 10000;

        double expectedDrop = Shooter.ShooterPID.expectedDrop * (0.25 * tx - 40);

        for (double velocity = 4000; velocity <= 8000; velocity += 100) {
            double a = Math.toDegrees(solveAngle(velocity,                       tx, ty));
            double b = Math.toDegrees(solveAngle(velocity - expectedDrop, tx, ty));
            double c = Math.toDegrees(solveAngle(velocity - expectedDrop * 2, tx, ty));

            if (a >= 30 && b >= 30 && c >= 30 && a <= 60 && b <= 60 && c <= 60) {
                double weight = velocity / Shooter.ShooterPID.velocityScale + Math.abs(c - a) / 20.0;
                if (weight < solutionWeight) {
                    solution = velocity;
                    solutionWeight = weight;
                }
            }
        }

        if (solution == 10000) return Double.NaN;
        return solution;
    }

    public static ProjectileMotion calculate(Vector3 target, double currentVelocity, double currentAngle) {

        Vector3 shooterPositionInField = new Vector3(
                SwerveDrive.PinpointCache.position.position.x(),
                SwerveDrive.PinpointCache.position.position.y(),
                Zeroing.ProjMotConstants.shooterOffset.getZ()
        );

        Vector3 relativeTarget = target.subtract(shooterPositionInField);
        double x = Math.hypot(relativeTarget.getX(), relativeTarget.getY());
        double distanceChange = Math.abs(ShootOnTheMoveConstants.calcDistance - x);

        ShootOnTheMoveConstants.lastDistance = x;
        double y = relativeTarget.getZ();

        if (far()) x += Shooter.ShooterPID.farZoneDistanceOffset;
        else x += Shooter.ShooterPID.closeZoneDistanceOffset;

        // Calculate the relative position of the target, on the ground, looking ahead in time to counteract turret lag.
        Vector2 groundPlane = Vector2.cartesian(relativeTarget.getX(), relativeTarget.getY())
                .sub(SwerveDrive.PinpointCache.velocity.multiply(ShootOnTheMoveConstants.turretLookahead));

        double rawDirection =
                groundPlane.polarDirection()
                        + 180
                        + SwerveDrive.PinpointCache.position.heading
                        + SwerveDrive.PinpointCache.angularVelocity * ShootOnTheMoveConstants.turretLookahead;

        // --- Normalize raw direction to [-180, 180] ---
        while (rawDirection > 180) rawDirection -= 360;
        while (rawDirection < -180) rawDirection += 360;

        // --- Choose equivalent closest to current turret angle ---
        double direction = rawDirection;

        while (direction - currentAngle > 180) direction -= 360;
        while (direction - currentAngle < -180) direction += 360;

        // --- Enforce turret mechanical limits (-210 to +210) ---
        if (direction > 210) direction -= 360;
        if (direction < -210) direction += 360;

        // Final safety clamp (guarantee never exceeding limits)
        direction = Math.max(-210, Math.min(210, direction));

        // Projectile math
        double velocity;

        if (distanceChange > 150){
            velocity = alternativeFindSuitableVelocity(x, y);
            ShootOnTheMoveConstants.calcDistance = x;
            ShootOnTheMoveConstants.lastVelocity = velocity;
        }
        else velocity = ShootOnTheMoveConstants.lastVelocity;

        if (!Shooter.ShooterPID.useVComp || currentVelocity < 1000) currentVelocity = velocity;
        double angle = solveAngle(currentVelocity, x, y);

        if (Double.isNaN(velocity)) velocity = 8500.0;
        if (Double.isNaN(angle)) angle = solveAngle(velocity, x, y);
        if (Double.isNaN(angle)) angle = Math.toRadians(45);

        if (x < 500) angle = Math.toRadians(60);

        // Given that x = vt cos (a), t = x / (v cos (a))
        if (currentVelocity < 2500) currentVelocity = velocity;
        double timeOfFlight = x / (currentVelocity * Math.cos(angle));

        return new ProjectileMotion(velocity, Math.toDegrees(angle), direction, timeOfFlight, true, x);
    }

    /**
     * Calculate the target parameters and iteratively converge on a perfect SOTM intersection
     * @param target The target Vector(right, forward, up), in mm
     * @param currentVelocity The current flywheel velocity, in mm
     * @return The aiming parameters that should work even accounting for robot velocity
     */
    public static ProjectileMotion calculateConvergence(Vector3 target, double currentVelocity, double currentAngle) {
        ProjectileMotion solution = calculate(target, currentVelocity, currentAngle);
        if (solution.groundDist < 800 || SwerveDrive.PinpointCache.velocity.magnitude() < 100) return solution;

        Vector3 expectedMotion;

        for (int convergence = 0; convergence < ShootOnTheMoveConstants.convergence; convergence++) {
            expectedMotion = new Vector3(SwerveDrive.PinpointCache.velocity.x(), SwerveDrive.PinpointCache.velocity.y(), 0.0)
                    .scale(solution.time * ShootOnTheMoveConstants.timeOverestimate);
            Vector3 trueTarget = target.subtract(expectedMotion);
            solution = calculate(trueTarget, currentVelocity, currentAngle);
        }

        return solution;
    }

    public static boolean far() {
        double distance = SwerveDrive.PinpointCache.position.position.sub( Vector2.cartesian(Shooter.ShooterPID.targetXFar, Shooter.ShooterPID.targetYFar) ).magnitude();
        return distance > 2500;
    }

    public static boolean mid() {
        double distance = SwerveDrive.PinpointCache.position.position.sub( Vector2.cartesian(Shooter.ShooterPID.targetXFar, Shooter.ShooterPID.targetYFar) ).magnitude();
        return distance > 1800;
    }

    public static Vector3 getTarget() {
        if (far()) {
            target.setX(Shooter.ShooterPID.targetXFar);
            target.setY(Shooter.ShooterPID.targetYFar);
            target.setZ(Shooter.ShooterPID.targetZFar);
        } else {
            target.setX(Shooter.ShooterPID.targetXClose);
            target.setY(Shooter.ShooterPID.targetYClose);
            if (mid()) {
                target.setZ(Shooter.ShooterPID.targetZMedium);
            } else {
                target.setZ(Shooter.ShooterPID.targetZClose);
            }
        } return target;
    }
}
