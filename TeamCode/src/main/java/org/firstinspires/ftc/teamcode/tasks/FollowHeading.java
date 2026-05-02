package org.firstinspires.ftc.teamcode.tasks;

import org.firstinspires.ftc.teamcode.lioncore.math.types.Path;
import org.firstinspires.ftc.teamcode.lioncore.math.types.Position;
import org.firstinspires.ftc.teamcode.lioncore.paths.Line;
import org.firstinspires.ftc.teamcode.lioncore.tasks.Task;
import org.firstinspires.ftc.teamcode.systems.Follower;
import org.firstinspires.ftc.teamcode.systems.SwerveDrive;

public class FollowHeading extends Task {
    private final Follower follower;
    private final Position end;
    private double heading;
    private double previousMaximumSpeed = 0;
    private double maxSpeed = Follower.FollowerConstants.maxSpeed;

    public FollowHeading(Follower follower, Position end) {
        this.follower = follower;
        this.end = end;
    }

    public FollowHeading setMaxSpeed(double speed) {
        this.maxSpeed = speed;
        return this;
    }

    @Override
    public void init() {
        Position start = SwerveDrive.PinpointCache.position.clone();
        this.previousMaximumSpeed = Follower.FollowerConstants.maxSpeed;
        double h = SwerveDrive.PinpointCache.position.heading;
        if (h > 90 || h < -90) this.heading = 180;
        else this.heading = 0;
        Follower.FollowerConstants.maxSpeed = this.maxSpeed;
        this.end.heading = this.heading;
        follower.follow(new Line(
            start,
            end
        ));
    }

    @Override
    // Finishes when the follower is within 15cm of the target.
    public boolean finished() {
        return follower.getDistance() < 150 || follower.driver();
    }

    @Override
    public void end(boolean i) {
        follower.stop();
        Follower.FollowerConstants.maxSpeed = this.previousMaximumSpeed;
    }
}
