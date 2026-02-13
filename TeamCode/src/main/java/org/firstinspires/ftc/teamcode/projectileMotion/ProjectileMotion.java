package org.firstinspires.ftc.teamcode.projectileMotion;
import org.firstinspires.ftc.teamcode.lioncore.math.types.Vector;

public class ProjectileMotion {

    public double flat = 0;
    public double arc = 0;
    public double flatTimeOfFlight = 0;
    public double arcTimeOfFlight = 0;

    public boolean flatPossible = false;
    public boolean arcPossible = false;

    public static double G = 9.8;

    private ProjectileMotion(double flat, double arc, double flatTimeOfFlight, double arcTimeOfFlight, boolean flatPossible, boolean arcPossible) {
        this.flat = flat;
        this.arc = arc;
        this.flatTimeOfFlight = flatTimeOfFlight;
        this.arcTimeOfFlight = arcTimeOfFlight;
        this.flatPossible = flatPossible;
        this.arcPossible = arcPossible;
    }

    public ProjectileMotion calculate(double launchVelocity, Vector position) {
        double v2 = Math.pow(launchVelocity, 2);
        double determinant = Math.pow(launchVelocity, 4) - G * (G * Math.pow(position.x(), 2) + 2 * position.y() * v2);

        if (determinant < 0) {
            return new ProjectileMotion(0, 0, 0, 0, false, false);
        }

        double sqrt = Math.sqrt(determinant);
        double gx = G * position.x();
        double flat = Math.toDegrees(Math.atan((v2 - sqrt) / gx));
        double arc = Math.toDegrees(Math.atan((v2 + sqrt) / gx));
        double flatTimeOfFlight = position.x() / (launchVelocity * Math.cos(Math.toRadians(flat)));
        double arcTimeOfFlight = position.x() / (launchVelocity * Math.cos(Math.toRadians(arc)));

        return new ProjectileMotion(flat, arc, flatTimeOfFlight, arcTimeOfFlight, true, true);
    }
}