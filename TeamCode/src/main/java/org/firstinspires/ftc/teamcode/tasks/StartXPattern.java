package org.firstinspires.ftc.teamcode.tasks;

import org.firstinspires.ftc.teamcode.lioncore.tasks.Task;
import org.firstinspires.ftc.teamcode.systems.Follower;

public class StartXPattern extends Task {
    private Follower swerveDrive;

    public StartXPattern(Follower swerveDrive) {
        this.swerveDrive = swerveDrive;
    }

    @Override
    public void init() {
        this.swerveDrive.setXPattern(true);
    }

    @Override
    public boolean finished() { return true; }
}