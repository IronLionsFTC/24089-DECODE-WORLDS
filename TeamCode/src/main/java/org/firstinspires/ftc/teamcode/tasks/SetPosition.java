package org.firstinspires.ftc.teamcode.tasks;

import org.firstinspires.ftc.teamcode.lioncore.math.types.Position;
import org.firstinspires.ftc.teamcode.lioncore.tasks.Task;
import org.firstinspires.ftc.teamcode.systems.Follower;

public class SetPosition extends Task {
    private Follower follower;
    private Position position;

    public SetPosition(Follower follower, Position position) {
        this.follower = follower;
        this.position = position;
    }

    @Override
    public void init() {
        this.follower.setPosition(position);
    }

    @Override
    public boolean finished() {
        return true;
    }
}
