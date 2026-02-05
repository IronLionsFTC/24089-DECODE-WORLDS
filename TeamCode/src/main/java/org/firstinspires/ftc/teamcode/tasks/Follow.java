package org.firstinspires.ftc.teamcode.tasks;

import org.firstinspires.ftc.teamcode.lioncore.math.types.Path;
import org.firstinspires.ftc.teamcode.lioncore.tasks.Task;
import org.firstinspires.ftc.teamcode.systems.Follower;

public class Follow extends Task {
    private final Follower follower;
    private final Path path;
    private final boolean holdEnd;

    public Follow(Follower follower, Path path) {
        this.follower = follower;
        this.path = path;
        this.holdEnd = false;
    }

    public Follow(Follower follower, Path path, boolean holdEnd) {
        this.follower = follower;
        this.path = path;
        this.holdEnd = holdEnd;
    }

    @Override
    public void init() {
        follower.follow(path);
    }

    @Override
    // Finishes when the follower is within 1cm of the target.
    public boolean finished() {
        return follower.getDistance() < 50;
    }

    @Override
    public void end(boolean i) {
        if (i || !holdEnd) follower.stop();
    }
}
