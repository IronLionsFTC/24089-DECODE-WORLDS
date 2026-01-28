package org.firstinspires.ftc.teamcode.lioncore.paths;

import org.firstinspires.ftc.teamcode.lioncore.math.types.Path;
import org.firstinspires.ftc.teamcode.lioncore.math.types.Position;
import org.firstinspires.ftc.teamcode.lioncore.math.types.Vector;

public class Line implements Path {

    private Position start;
    private Position end;

    @Override
    public double getClosestK(Vector position) {
        return 0.0;
    }

    @Override
    public Position getTarget(double k) {
        Vector direction = end.position.sub(start.position);
        Vector interpolation = start.position.add(direction.multiply(k));
        return new Position(interpolation.x(), interpolation.y(), 0);
    }

    @Override
    public double distanceRemaining(double k) {
        return 0.0;
    }
}
