package org.firstinspires.ftc.teamcode.tasks;

import org.firstinspires.ftc.teamcode.lioncore.math.types.Path;
import org.firstinspires.ftc.teamcode.lioncore.tasks.Task;
import org.firstinspires.ftc.teamcode.systems.Follower;

public class Follow extends Task {
    private final Follower follower;
    private final Path path;

    private double previousMaximumSpeed = 0;
    private double maxSpeed = Follower.FollowerConstants.maxSpeed;

    public Follow(Follower follower, Path path) {
        this.follower = follower;
        this.path = path;
    }

    public Follow setMaxSpeed(double speed) {
        this.maxSpeed = speed;
        return this;
    }

    @Override
    public void init() {
        this.previousMaximumSpeed = Follower.FollowerConstants.maxSpeed;
        Follower.FollowerConstants.maxSpeed = this.maxSpeed;
        follower.follow(path);
    }

    @Override
    // Finishes when the follower is within 1cm of the target.
    public boolean finished() {
        return follower.getDistance() < 50;
    }

    @Override
    public void end(boolean i) {
        follower.stop();
        Follower.FollowerConstants.maxSpeed = this.previousMaximumSpeed;
    }
}
