package org.firstinspires.ftc.teamcode.tasks;

import org.firstinspires.ftc.teamcode.lioncore.math.types.Path;
import org.firstinspires.ftc.teamcode.lioncore.math.types.Position;
import org.firstinspires.ftc.teamcode.lioncore.paths.Line;
import org.firstinspires.ftc.teamcode.lioncore.tasks.Task;
import org.firstinspires.ftc.teamcode.systems.Follower;
import org.firstinspires.ftc.teamcode.systems.SwerveDrive;

public class Goto extends Task {
    private final Follower follower;
    private final Position target;

    private double previousMaximumSpeed = 0;
    private double maxSpeed = Follower.FollowerConstants.maxSpeed;

    public Goto(Follower follower, Position target) {
        this.follower = follower;
        this.target = target;
    }

    public Goto setMaxSpeed(double speed) {
        this.maxSpeed = speed;
        return this;
    }

    @Override
    public void init() {
        this.previousMaximumSpeed = Follower.FollowerConstants.maxSpeed;
        Follower.FollowerConstants.maxSpeed = this.maxSpeed;
        follower.follow(new Line(
                SwerveDrive.PinpointCache.position,
                target
        ));
    }

    @Override
    // Finishes when the follower is within 15cm of the target.
    public boolean finished() {
        return follower.getDistance() < 100 || follower.driver();
    }

    @Override
    public void end(boolean i) {
        follower.stop();
        Follower.FollowerConstants.maxSpeed = this.previousMaximumSpeed;
    }
}
