package org.firstinspires.ftc.teamcode.lioncore.math.types;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;

public class Vector {
    private double x;
    private double y;

    private Vector(double x, double y) {
        this.x = x;
        this.y = y;
    }

    /**
     * Create a Vector from the x and y cartesian components
     * @param x x component
     * @param y y component
     * @return Component vector
     */
    public static Vector cartesian(double x, double y) {
        return new Vector(x, y);
    }

    /**
     * Create a Vector from the magnitude and direction
     * @param m Magnitude
     * @param d Direction in RADIANS
     * @return Component vector
     */
    public static Vector polar(double m, double d) {
        return new Vector(Math.cos(d), Math.sin(d)).multiply(m);
    }

    public double x() { return this.x; }
    public double y() { return this.y; }

    /**
     * Get the direction of the vector
     * @return Direction in radians
     */
    public double direction() {
        return Math.atan2(this.y, this.x);
    }

    public double polarDirection() {
        return Math.toDegrees(Math.atan2(this.x, this.y));
    }

    public double magnitude() { return Math.sqrt(Math.pow(this.x, 2.0) + Math.pow(this.y, 2.0)); }

    public Vector multiply(double scalar) {
        return new Vector(this.x * scalar, this.y * scalar);
    }

    public void multiply_mut(double scalar) {
        this.x *= scalar;
        this.y *= scalar;
    }

    public Vector add(Vector other) {
        return new Vector(this.x + other.x, this.y + other.y);
    }

    public Vector sub(Vector other) {
        return new Vector(this.x - other.x, this.y - other.y);
    }

    public void add_mut(Vector other) {
        this.x += other.x;
        this.y += other.y;
    }

    public Pose2D pose(double heading) {
        return new Pose2D(DistanceUnit.MM, this.x, this.y, AngleUnit.DEGREES, heading);
    }

    public Vector normalised() {
        return new Vector(this.x / this.magnitude(), this.y / this.magnitude());
    }
}
