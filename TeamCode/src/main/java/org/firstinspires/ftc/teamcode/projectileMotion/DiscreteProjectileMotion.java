package org.firstinspires.ftc.teamcode.projectileMotion;

import com.acmerobotics.dashboard.config.Config;
import org.firstinspires.ftc.teamcode.lioncore.math.types.Vector2;
import org.firstinspires.ftc.teamcode.lioncore.math.types.Vector3;
import org.firstinspires.ftc.teamcode.parameters.Zeroing;
import org.firstinspires.ftc.teamcode.systems.SwerveDrive;

public class DiscreteProjectileMotion {

    @Config
    public static class DPM {
        public static double G = 9800;
        public static double MINANGLE = 30.0;
        public static double MAXANGLE = 52.0;
        public static double MINVELOCITY = 6000.0;
        public static double MAXVELOCITY = 8800.0;
        public static double FINEANGLE = 1;
        public static double COARSEANGLE = 3;
        public static double VSTEPCOARSE = 250;
        public static double LOSS = 0;

        public static double VELOCITYSCALE = 200;
        public static double MAXHEIGHTSCALE = 100;
        public static double DIFSCALE = 1;
    }

    public static class Aiming {
        public Solution solution;
        public double azimuth;
        public double velocity;

        public Aiming(Solution theoretical, Solution actual, double azimuth) {
            this.solution = actual;
            this.azimuth = azimuth;
            this.velocity = theoretical.velocity;
        }
    }

    public static class Solution {
        public double velocity;
        public double altitude;
        public double time;
        public double error;
        public double maxHeight;

        public Solution(double velocity, double altitude, Vector2 target) {
            this.velocity = velocity;
            this.altitude = altitude;

            double targetX = target.x();
            double targetY = target.y();

            double rad = Math.toRadians(altitude);

            double vx = velocity * Math.cos(rad);
            double vy = velocity * Math.sin(rad);

            // If horizontal velocity is zero, the projectile never reaches the target
            if (Math.abs(vx) < 1e-6) {
                this.time = Double.POSITIVE_INFINITY;
                this.error = Double.POSITIVE_INFINITY;
                this.maxHeight = 0;
                return;
            }

            // Time to reach the target X distance
            double time = targetX / vx;

            // Height at that time
            double y = vy * time - 0.5 * DPM.G * time * time;

            // Maximum height
            double maxHeight = (vy * vy) / (2 * DPM.G);

            this.time = time;
            this.maxHeight = maxHeight;

            this.error = targetY - y;
            if (this.error < 0) {
                this.error *= -3;
            }
        }

        public double rank() {
            return this.error + this.velocity / DPM.VELOCITYSCALE + this.maxHeight / DPM.MAXHEIGHTSCALE;
        }

    }

    public static Solution withVelocity(double velocity, Vector2 target) {
        Solution bestSolution = null;
        for (double angle = DPM.MINANGLE; angle <= DPM.MAXANGLE; angle += DPM.COARSEANGLE) {
            Solution solution = new Solution(velocity, angle, target);
            if (bestSolution == null) {
                bestSolution = solution;
            } else if (solution.rank() < bestSolution.rank()) {
                bestSolution = solution;
            }
        }

        if (bestSolution == null) return null;

        for (double angle = bestSolution.altitude - 2; angle < bestSolution.altitude + 2; angle += DPM.FINEANGLE) {
            Solution solution = new Solution(velocity, angle, target);
            if (solution.rank() < bestSolution.rank()) {
                bestSolution = solution;
            }
        }

        return bestSolution;
    }

    public static Solution optimalLaunchVelocity(Vector2 target) {

        double bestRank = 0;
        Solution optimalSolution = null;

        for (double velocity = DPM.MINVELOCITY; velocity < DPM.MAXVELOCITY; velocity += DPM.VSTEPCOARSE) {
            Solution solution = withVelocity(velocity, target);
            Solution solutionLoss = withVelocity(velocity - DPM.LOSS, target);
            if (solution == null || solutionLoss == null) continue;
            double rank = solution.rank()
                    + solutionLoss.rank()
                    + Math.abs(solution.altitude - solutionLoss.altitude) * DPM.DIFSCALE;

            if (rank < bestRank || bestRank == 0) {
                bestRank = rank;
                optimalSolution = solution;
            }
        }

        return optimalSolution;
    }

    public static Aiming solve(Vector3 target, double measuredVelocity) {
        // Robot heading in radians
        double headingRad = Math.toRadians(SwerveDrive.PinpointCache.position.heading);

        // Turret offset in robot frame
        Vector3 shooterOffset = Zeroing.ProjMotConstants.shooterOffset;

        // Rotate offset into field frame
        double rotatedX = shooterOffset.getX() * Math.cos(headingRad)
                - shooterOffset.getY() * Math.sin(headingRad);

        double rotatedY = shooterOffset.getX() * Math.sin(headingRad)
                + shooterOffset.getY() * Math.cos(headingRad);

        // Shooter position in field frame
        Vector3 shooterPositionInField = new Vector3(
                SwerveDrive.PinpointCache.position.position.x() + rotatedX,
                SwerveDrive.PinpointCache.position.position.y() + rotatedY,
                shooterOffset.getZ()
        );

        // Vector from turret to target
        Vector3 relativeTarget = target.subtract(shooterPositionInField);

        // Calculate the relative position of the target, on the ground, looking ahead in time to counteract turret lag.
        Vector2 groundPlane = Vector2.cartesian(relativeTarget.getX(), relativeTarget.getY())
                .sub(SwerveDrive.PinpointCache.velocity.multiply(ProjectileMotion.ShootOnTheMoveConstants.turretLookahead));
        double direction = 180 + groundPlane.polarDirection()
                + SwerveDrive.PinpointCache.position.heading
                + SwerveDrive.PinpointCache.angularVelocity * ProjectileMotion.ShootOnTheMoveConstants.turretLookahead;
        while (direction < -180) direction += 360;
        while (direction >  180) direction -= 360;

        // The 2D aiming vector for angle / velocity
        Vector2 cartesian = Vector2.cartesian(groundPlane.magnitude(), relativeTarget.getZ());
        Solution theoretical = optimalLaunchVelocity(cartesian);
        if (measuredVelocity == 0) measuredVelocity = theoretical.velocity;
        Solution actual = withVelocity(measuredVelocity, cartesian);

        return new Aiming(theoretical, actual, direction);
    }

    public static Aiming solveConvergent(Vector3 target, double measuredVelocity) {
        Aiming solution = solve(target, measuredVelocity);
        Vector3 expectedMotion;

        for (int convergence = 0; convergence < ProjectileMotion.ShootOnTheMoveConstants.convergence; convergence++) {
            expectedMotion = new Vector3(SwerveDrive.PinpointCache.velocity.x(), SwerveDrive.PinpointCache.velocity.y(), 0.0)
                    .scale(solution.solution.time * ProjectileMotion.ShootOnTheMoveConstants.timeOverestimate);
            Vector3 trueTarget = target.subtract(expectedMotion);
            solution = solve(trueTarget, measuredVelocity);
        }

        return solution;
    }
}