package org.firstinspires.ftc.teamcode.tasks;

import org.firstinspires.ftc.teamcode.lioncore.tasks.Task;
import org.firstinspires.ftc.teamcode.systems.SwerveDrive;

public class EndXPattern extends Task {
    private SwerveDrive swerveDrive;

    public EndXPattern(SwerveDrive swerveDrive) {
        this.swerveDrive = swerveDrive;
    }

    @Override
    public void init() {
        this.swerveDrive.setXPattern(true);
    }

    @Override
    public boolean finished() { return true; }
}