package org.firstinspires.ftc.teamcode.tasks;

import org.firstinspires.ftc.teamcode.lioncore.tasks.Task;
import org.firstinspires.ftc.teamcode.systems.Follower;

public class RelocaliseToStart extends Task {
    private Follower swerveDrive;

    public RelocaliseToStart(Follower swerveDrive) {
        this.swerveDrive = swerveDrive;
    }

    @Override
    public void init() {
        this.swerveDrive.relocalise();
    }

    @Override
    public boolean finished() {
        return true;
    }
}
