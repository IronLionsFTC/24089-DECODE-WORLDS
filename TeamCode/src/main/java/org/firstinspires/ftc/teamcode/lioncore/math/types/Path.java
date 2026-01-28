package org.firstinspires.ftc.teamcode.lioncore.math.types;

public interface Path {

    /**
     * Return the value of the parametric variable 'k' which represents the position along this path from 0->1
     * @param position The current position of the robot
     * @return
     */
    double getClosestK(Vector position);

    /**
     * Return the next targetted position on the path
     * @param k [0, 1]
     * @return
     */
    Position getTarget(double k);

    /**
     * Return the distance remaining on a path given the k value. Used for drive power calculations
     * @param k The K value defining the point on the curve
     * @return The distance remaining in mm
     */
    double distanceRemaining(double k);
}
