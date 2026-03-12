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
        public static double T = 0.02;
        public static double MINANGLE = 20.0;
        public static double MAXANGLE = 52.0;
        public static double MINVELOCITY = 5000.0;
        public static double MAXVELOCITY = 10000.0;
        public static double FINEANGLE = 0.1;
        public static double COARSEANGLE = 1;
        public static double VSTEP = 100;
        public static double LOSS = 0;
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

            double time = 0;
            double maxHeight = 0;
            Vector2 positionVec = Vector2.cartesian(0, 0);
            Vector2 velocityVec = Vector2.polar(this.velocity, Math.toRadians(this.altitude));
            Vector2 acceleration = Vector2.cartesian(0, -DPM.G);

            while (positionVec.x() < target.x()) {
                time += DPM.T;
                velocityVec.add(acceleration.multiply(DPM.T));
                positionVec.add(velocityVec.multiply(DPM.T));
                if (positionVec.y() > maxHeight) maxHeight = positionVec.y();
            }

            this.error = target.y() - positionVec.y();
            this.time = time;
            this.maxHeight = maxHeight;
            if (this.error < 0) this.error = this.error * -3;
        }

        public double rank() {
            return this.error + this.velocity / 200 + this.maxHeight / 100;
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

        for (double velocity = DPM.MINVELOCITY; velocity < DPM.MAXVELOCITY; velocity += DPM.VSTEP) {
            Solution solution = withVelocity(velocity, target);
            Solution solutionLoss = withVelocity(velocity - DPM.LOSS, target);
            if (solution == null || solutionLoss == null) continue;
            double rank = solution.rank()
                    + solutionLoss.rank()
                    + Math.abs(solution.altitude - solutionLoss.altitude);

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