package org.firstinspires.ftc.teamcode.projectileMotion;
import org.firstinspires.ftc.teamcode.lioncore.math.types.Vector2;
import org.firstinspires.ftc.teamcode.lioncore.math.types.Vector3;

public class ProjectileMotion {

    public static class ProjMotConstants {
        public static Vector3 shooterOffset = new Vector3(0, -20, 300);
    }

    public double flat = 0;
    public double arc = 0;
    public double flatTimeOfFlight = 0;
    public double arcTimeOfFlight = 0;

    public boolean flatPossible = false;
    public boolean arcPossible = false;

    public static double G = -9800;

    private ProjectileMotion(double flat, double arc, double flatTimeOfFlight, double arcTimeOfFlight, boolean flatPossible, boolean arcPossible) {
        this.flat = flat;
        this.arc = arc;
        this.flatTimeOfFlight = flatTimeOfFlight;
        this.arcTimeOfFlight = arcTimeOfFlight;
        this.flatPossible = flatPossible;
        this.arcPossible = arcPossible;
    }

    public static ProjectileMotion calculate(double launchVelocity, Vector2 position) {
        double x = position.x();
        double y = position.y();
        double v2 = Math.pow(launchVelocity, 2);
        double g2 = Math.pow(G, 2);
        double determinant = Math.pow(y * G - v2, 2) - g2 * (x*x + y*y);

        if (determinant < 0) {
            return new ProjectileMotion(0, 0, 0, 0, false, false);
        }

        double sqrt = Math.sqrt(determinant);

        double flat = ((v2 - y * G) - sqrt) / g2;
        double arc = ((v2 - y * G) + sqrt) / g2;

        double flatTimeOfFlight = Math.sqrt(flat);
        double arcTimeOfFlight = Math.sqrt(arc);

        flat = Math.toDegrees(Math.acos(x / (launchVelocity * flatTimeOfFlight)));
        arc = Math.toDegrees(Math.acos(x / (launchVelocity * arcTimeOfFlight)));

        return new ProjectileMotion(flat, arc, flatTimeOfFlight, arcTimeOfFlight, !Double.isNaN(flat), !Double.isNaN(arc));
    }

    /**
     * Calculates the required launch velocity to achieve a target average speed
     * over the time of flight, taking air resistance into account.
     *
     * @param targetAverageSpeed desired average speed (mm/s)
     * @param timeOfFlight       total time of flight (s)
     * @return required launch velocity (m/s)
     */
    public static double calculateRequiredLaunchVelocity(double targetAverageSpeed, double timeOfFlight) {
        targetAverageSpeed /= 1000;
        // Constants for a 5-inch diameter sphere
        final double AIR_DENSITY = 1.225;  // kg/m^3
        final double DRAG_COEFF = 0.47;    // sphere
        final double DIAMETER_METERS = 0.127; // 5 inches in meters

        // Calculate cross-sectional area of the ball (A = pi * r^2)
        double radius = DIAMETER_METERS / 2.0;
        double area = Math.PI * radius * radius;

        // Calculate the drag constant beta
        double beta = 0.5 * AIR_DENSITY * DRAG_COEFF * area / 0.09;

        // Calculate the required launch velocity
        double exponent = beta * timeOfFlight * targetAverageSpeed;
        return ((Math.exp(exponent) - 1.0) / (beta * timeOfFlight)) * 1000;
    }


    public static double calculateAverageSpeedFromLaunchVelocity(double launchVelocity, double timeOfFlight) {
        launchVelocity /= 1000;

        // Constants for a 5-inch diameter sphere
        final double AIR_DENSITY = 1.225;  // kg/m^3
        final double DRAG_COEFF = 0.47;    // sphere
        final double DIAMETER_METERS = 0.127; // 5 inches in meters

        // Calculate cross-sectional area of the ball (A = pi * r^2)
        double radius = DIAMETER_METERS / 2.0;
        double area = Math.PI * radius * radius;

        // Calculate the drag constant beta
        double beta = 0.5 * AIR_DENSITY * DRAG_COEFF * area / 0.09;

        // Calculate the target average speed using the inverse of the original formula
        double exponentTerm = Math.log((beta * timeOfFlight * launchVelocity) + 1);
        return (exponentTerm / (beta * timeOfFlight)) * 1000;
    }
}