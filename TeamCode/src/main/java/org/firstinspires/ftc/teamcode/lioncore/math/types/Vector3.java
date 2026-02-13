package org.firstinspires.ftc.teamcode.lioncore.math.types;

public class Vector3 {
    private double x, y, z;

    // Constructor
    public Vector3(double x, double y, double z) {
        this.x = x;
        this.y = y;
        this.z = z;
    }

    // Getter methods
    public double getX() {
        return x;
    }

    public double getY() {
        return y;
    }

    public double getZ() {
        return z;
    }

    // Setter methods
    public void setX(double x) {
        this.x = x;
    }

    public void setY(double y) {
        this.y = y;
    }

    public void setZ(double z) {
        this.z = z;
    }

    // Add another vector
    public Vector3 add(Vector3 v) {
        return new Vector3(this.x + v.x, this.y + v.y, this.z + v.z);
    }

    // Subtract another vector
    public Vector3 subtract(Vector3 v) {
        return new Vector3(this.x - v.x, this.y - v.y, this.z - v.z);
    }

    // Multiply by a scalar
    public Vector3 scale(double scalar) {
        return new Vector3(this.x * scalar, this.y * scalar, this.z * scalar);
    }

    // Dot product
    public double dot(Vector3 v) {
        return this.x * v.x + this.y * v.y + this.z * v.z;
    }

    // Cross product
    public Vector3 cross(Vector3 v) {
        return new Vector3(
                this.y * v.z - this.z * v.y,
                this.z * v.x - this.x * v.z,
                this.x * v.y - this.y * v.x
        );
    }

    // Magnitude (length)
    public double magnitude() {
        return Math.sqrt(x * x + y * y + z * z);
    }

    // Normalize the vector (unit vector)
    public Vector3 normalize() {
        double length = magnitude();
        if (length == 0) {
            throw new ArithmeticException("Cannot normalize a zero vector.");
        }
        return scale(1.0 / length);
    }

    // Angle between two vectors (in radians)
    public double angleBetween(Vector3 v) {
        double dotProduct = dot(v);
        double magnitudes = this.magnitude() * v.magnitude();
        if (magnitudes == 0) {
            throw new ArithmeticException("Cannot calculate the angle with a zero vector.");
        }
        return Math.acos(dotProduct / magnitudes);
    }

    // Distance between two vectors
    public double distance(Vector3 v) {
        return Math.sqrt(Math.pow(this.x - v.x, 2) + Math.pow(this.y - v.y, 2) + Math.pow(this.z - v.z, 2));
    }

    // Reflect the vector off a surface defined by a normal vector
    public Vector3 reflect(Vector3 normal) {
        Vector3 normalizedNormal = normal.normalize();
        double dotProduct = dot(normalizedNormal);
        return subtract(normalizedNormal.scale(2 * dotProduct));
    }

    // Linearly interpolate between two vectors (lerp)
    public Vector3 lerp(Vector3 v, double t) {
        return new Vector3(
                this.x + t * (v.x - this.x),
                this.y + t * (v.y - this.y),
                this.z + t * (v.z - this.z)
        );
    }

    // To string representation
    @Override
    public String toString() {
        return String.format("Vector3(%f, %f, %f)", x, y, z);
    }

    // Static methods for common vector operations

    // Create a zero vector
    public static Vector3 zero() {
        return new Vector3(0, 0, 0);
    }

    // Create a unit vector along the X axis
    public static Vector3 unitX() {
        return new Vector3(1, 0, 0);
    }

    // Create a unit vector along the Y axis
    public static Vector3 unitY() {
        return new Vector3(0, 1, 0);
    }

    // Create a unit vector along the Z axis
    public static Vector3 unitZ() {
        return new Vector3(0, 0, 1);
    }

    // Linear interpolation between two vectors (static method)
    public static Vector3 lerp(Vector3 start, Vector3 end, double t) {
        return new Vector3(
                start.x + t * (end.x - start.x),
                start.y + t * (end.y - start.y),
                start.z + t * (end.z - start.z)
        );
    }

    // Find the distance between two vectors (static method)
    public static double distance(Vector3 v1, Vector3 v2) {
        return Math.sqrt(Math.pow(v1.x - v2.x, 2) + Math.pow(v1.y - v2.y, 2) + Math.pow(v1.z - v2.z, 2));
    }

    // Find the angle between two vectors (static method)
    public static double angleBetween(Vector3 v1, Vector3 v2) {
        double dotProduct = v1.dot(v2);
        double magnitudes = v1.magnitude() * v2.magnitude();
        if (magnitudes == 0) {
            throw new ArithmeticException("Cannot calculate the angle with a zero vector.");
        }
        return Math.acos(dotProduct / magnitudes);
    }

    // Check if two vectors are equal (within a tolerance)
    public boolean equals(Vector3 v, double tolerance) {
        return Math.abs(this.x - v.x) <= tolerance && Math.abs(this.y - v.y) <= tolerance && Math.abs(this.z - v.z) <= tolerance;
    }
}
