package org.firstinspires.ftc.teamcode.lioncore.math.types;

public interface Path {

    /**
     * Return the value of the parametric variable 'k' which represents the position along this path from 0->1
     * @return
     */
    double getClosestK();

    /**
     * Return the position on the path given by a k value
     * @param k [0, 1]
     * @return
     */
    void getTarget(double k, Position output);

    /**
     * Return the distance remaining on a path based on the robot position.
     * It is at the Path implementations discretion whether this includes the error of the robot from the path.
     * @return The distance remaining in mm
     */
    double distanceRemaining();
}
