package org.firstinspires.ftc.teamcode.lioncore.math.types;

import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;

public class Position {

    public Vector position;
    public double heading;

    public Position(double x, double y, double heading) {
        this.position = Vector.cartesian(x, y);
        this.heading = heading;
    }

    public Pose2D pose() {
        return this.position.pose(this.heading);
    }

    public void update(double x, double y, double heading) {
        this.position = Vector.cartesian(x, y);
        this.heading = heading;
    }
}
