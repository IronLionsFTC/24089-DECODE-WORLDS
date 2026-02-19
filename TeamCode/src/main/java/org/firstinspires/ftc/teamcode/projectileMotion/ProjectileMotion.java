package org.firstinspires.ftc.teamcode.projectileMotion;
import org.firstinspires.ftc.teamcode.lioncore.math.types.Vector2;
import org.firstinspires.ftc.teamcode.lioncore.math.types.Vector3;

public class ProjectileMotion {

    public static class ProjMotConstants {
        public static Vector3 shooterOffset = new Vector3(0, -20, 300);
    }

    public final boolean hasSolution;
    public final double  angle;
    public final Vector2 velocity;

    private ProjectileMotion(boolean hasSolution, double angle, Vector2 velocity) {
        this.hasSolution = hasSolution;
        this.angle       = angle;
        this.velocity    = velocity;
    }

    private static final double G         = 9800.0;
    private static final double ANGLE_MIN = 45.0;
    private static final double ANGLE_MAX = 80.0;

    public static ProjectileMotion solve(Vector2 delta) {
        return solveConstrained(delta, ANGLE_MIN, ANGLE_MAX);
    }

    public static ProjectileMotion solve(Vector2 delta, double minAngle, double maxAngle) {
        return solveConstrained(delta, minAngle, maxAngle);
    }

    private static double requiredVelocityForAngle(Vector2 delta, double angleDeg) {
        double x     = delta.x();
        double y     = delta.y();
        double theta = Math.toRadians(angleDeg);
        double cos   = Math.cos(theta);
        double denom = 2.0 * (cos * cos) * (x * Math.tan(theta) - y);
        if (denom <= 0) return Double.NaN;
        double v2 = (G * x * x) / denom;
        if (v2 <= 0) return Double.NaN;
        return Math.sqrt(v2);
    }

    private static ProjectileMotion solveConstrained(Vector2 delta, double minAngle, double maxAngle) {
        double x = delta.x();
        double y = delta.y();

        if (x == 0) return noSolution();

        double r     = Math.sqrt(x * x + y * y);
        // t = tan(theta); low-energy (flat) shot first, high-energy (steep) shot as fallback
        double tLow  = (y - r) / x;
        double tHigh = (y + r) / x;

        ProjectileMotion result = evaluate(delta, tLow,  minAngle, maxAngle);
        if (result == null) result = evaluate(delta, tHigh, minAngle, maxAngle);
        if (result != null) return result;

        // Both unconstrained optima are outside the angle bounds — check boundaries
        double bestSpeed = Double.NaN;
        double bestAngle = Double.NaN;
        for (double boundary : new double[]{ minAngle, maxAngle }) {
            double v = requiredVelocityForAngle(delta, boundary);
            if (!Double.isNaN(v) && (Double.isNaN(bestSpeed) || v < bestSpeed)) {
                bestSpeed = v;
                bestAngle = boundary;
            }
        }

        if (Double.isNaN(bestSpeed)) return noSolution();
        return solution(bestAngle, bestSpeed);
    }

    private static ProjectileMotion evaluate(Vector2 delta, double t, double minAngle, double maxAngle) {
        double angleDeg = Math.toDegrees(Math.atan(t));
        if (angleDeg < minAngle || angleDeg > maxAngle) return null;
        double v = requiredVelocityForAngle(delta, angleDeg);
        if (Double.isNaN(v)) return null;
        return solution(angleDeg, v);
    }

    private static ProjectileMotion solution(double angleDeg, double speed) {
        // Vector2.polar expects radians
        Vector2 velocity = Vector2.polar(speed, Math.toRadians(angleDeg));
        return new ProjectileMotion(true, angleDeg, velocity);
    }

    private static ProjectileMotion noSolution() {
        return new ProjectileMotion(false, Double.NaN, Vector2.cartesian(Double.NaN, Double.NaN));
    }
}
