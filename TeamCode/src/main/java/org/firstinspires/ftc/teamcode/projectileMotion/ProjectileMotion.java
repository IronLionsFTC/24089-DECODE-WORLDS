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
    public double timeOfFlight;

    private static final Vector3 target = Vector3.zero();

    public ProjectileMotion(double velocity, double angle, double yaw, double timeOfFlight, boolean possible) {
        this.launchVelocity = velocity;
        this.launchAngle = angle;
        this.yawDirection = yaw;
        this.timeOfFlight = timeOfFlight;
        this.possible = possible;
    }

    @Config
    public static class ShootOnTheMoveConstants {
        public static double turretLookahead = 0.1;
        public static int convergence = 15;
        public static double timeOverestimate = 1.3;
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
        return Math.min(plus, minu);
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

        for (double angle = 35; angle < 57.0; angle += 1.0) {
            double velocity = solveVelocity(Math.toRadians(angle), tx, ty);
            if (Double.isNaN(velocity)) continue;
            if (velocity > 9000) continue;

            double a = Math.toDegrees(solveAngle(velocity,                       tx, ty));
            double b = Math.toDegrees(solveAngle(velocity - Shooter.ShooterPID.expectedDrop, tx, ty));
            double c = Math.toDegrees(solveAngle(velocity - Shooter.ShooterPID.expectedDrop * 1.5, tx, ty));

            if (a >= 34 && b >= 34 && c >= 34 && a <= 57 && b <= 57 && c <= 57) {
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

    public static ProjectileMotion calculate(Vector3 target, double currentVelocity) {
        double cosHeading = Math.cos(SwerveDrive.PinpointCache.position.heading);
        double sinHeading = Math.sin(SwerveDrive.PinpointCache.position.heading);

        /*
        Vector3 shooterPositionInField = new Vector3(
                SwerveDrive.PinpointCache.position.position.x()
                        + Zeroing.ProjMotConstants.shooterOffset.getX() * cosHeading
                        - Zeroing.ProjMotConstants.shooterOffset.getY() * sinHeading,
                SwerveDrive.PinpointCache.position.position.y()
                        + Zeroing.ProjMotConstants.shooterOffset.getX() * sinHeading
                        + Zeroing.ProjMotConstants.shooterOffset.getY() * cosHeading,
                Zeroing.ProjMotConstants.shooterOffset.getZ()
        );*/
        Vector3 shooterPositionInField = new Vector3(
                SwerveDrive.PinpointCache.position.position.x(),
                SwerveDrive.PinpointCache.position.position.y(),
                Zeroing.ProjMotConstants.shooterOffset.getZ()
        );

        Vector3 relativeTarget = target.subtract(shooterPositionInField);
        double x = Math.hypot(relativeTarget.getX(), relativeTarget.getY());
        double y = relativeTarget.getZ();

        // Calculate the relative position of the target, on the ground, looking ahead in time to counteract turret lag.
        Vector2 groundPlane = Vector2.cartesian(relativeTarget.getX(), relativeTarget.getY())
                .sub(SwerveDrive.PinpointCache.velocity.multiply(ShootOnTheMoveConstants.turretLookahead));
        double direction = 180 + groundPlane.polarDirection()
                + SwerveDrive.PinpointCache.position.heading
                + SwerveDrive.PinpointCache.angularVelocity * ShootOnTheMoveConstants.turretLookahead;
        while (direction < -180) direction += 360;
        while (direction >  180) direction -= 360;


        // Projectile math
        double angle = solveAngle(currentVelocity, x, y);
        double velocity = findSuitableVelocity(x, y);

        if (Double.isNaN(velocity)) velocity = 7000.0;
        if (Double.isNaN(angle)) angle = solveAngle(velocity, x, y);
        if (Double.isNaN(angle)) angle = Math.toRadians(55);

        // Given that x = vt cos (a), t = x / (v cos (a))
        double timeOfFlight = x / (currentVelocity * Math.cos(angle));
        return new ProjectileMotion(velocity, Math.toDegrees(angle), direction, timeOfFlight, true);
    }

    /**
     * Calculate the target parameters and iteratively converge on a perfect SOTM intersection
     * @param target The target Vector(right, forward, up), in mm
     * @param currentVelocity The current flywheel velocity, in mm
     * @return The aiming parameters that should work even accounting for robot velocity
     */
    public static ProjectileMotion calculateConvergence(Vector3 target, double currentVelocity) {
        ProjectileMotion solution = calculate(target, currentVelocity);
        Vector3 expectedMotion;

        for (int convergence = 0; convergence < ShootOnTheMoveConstants.convergence; convergence++) {
            expectedMotion = new Vector3(SwerveDrive.PinpointCache.velocity.x(), SwerveDrive.PinpointCache.velocity.y(), 0.0)
                    .scale(solution.timeOfFlight * ShootOnTheMoveConstants.timeOverestimate);
            Vector3 trueTarget = target.subtract(expectedMotion);
            solution = calculate(trueTarget, currentVelocity);
        }

        double headingOffset = SwerveDrive.PinpointCache.velocity.magnitude() / 500;
        solution.launchAngle += headingOffset;

        return solution;
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
