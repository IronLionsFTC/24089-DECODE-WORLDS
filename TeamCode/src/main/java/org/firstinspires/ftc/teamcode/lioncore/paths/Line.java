package org.firstinspires.ftc.teamcode.lioncore.paths;

import org.firstinspires.ftc.teamcode.lioncore.math.types.Path;
import org.firstinspires.ftc.teamcode.lioncore.math.types.Position;
import org.firstinspires.ftc.teamcode.lioncore.math.types.Vector;
import org.firstinspires.ftc.teamcode.systems.SwerveDrive;

public class Line implements Path {

    private final Position start;
    private final Position end;
    private final Vector direction;
    private final Vector normalisedDirection;
    private final double length;
    private final Vector to_robot;
    private final Vector temp;
    public final Position target;

    public Line(Position start, Position end) {
        this.start = start;
        this.end = end;
        this.direction = this.end.position.sub(this.start.position);
        this.normalisedDirection = direction.normalised();
        this.to_robot = Vector.cartesian(0, 0);
        this.length = this.direction.magnitude();
        this.temp = Vector.cartesian(0, 0);
        this.target = new Position(
                start.position.x(),
                start.position.y(),
                start.heading
        );
    }

    @Override
    public double getClosestK() {
        SwerveDrive.PinpointCache.position.position.sub_into(start.position, to_robot);
        double projection = to_robot.dot(normalisedDirection);
        return Math.max(Math.min(projection / length, 1), 0);
    }

    @Override
    public Position getTarget(double k) {
        direction.multiply_into(k, temp);
        start.position.add_into(temp, temp);
        this.target.update(
                temp.x(), temp.y(),
                start.heading + (end.heading - start.heading) * k
        );

        return this.target;
    }

    @Override
    public double distanceRemaining() {
        end.position.sub_into(SwerveDrive.PinpointCache.position.position, temp);
        return temp.magnitude();
    }
}
