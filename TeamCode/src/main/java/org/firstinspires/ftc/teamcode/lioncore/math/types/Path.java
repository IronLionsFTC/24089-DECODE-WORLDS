package org.firstinspires.ftc.teamcode.lioncore.math.types;

public interface Path {

    /**
     * Return the value of the parametric variable 'k' which represents the position along this path from 0->1
     * @return
     */
    double getClosestK();

    /**
     * Return the next targetted position on the path
     * @param k [0, 1]
     * @return
     */
    Position getTarget(double k);
}
