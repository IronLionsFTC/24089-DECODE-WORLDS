package org.firstinspires.ftc.teamcode.tasks;

import org.firstinspires.ftc.teamcode.lioncore.tasks.Task;
import org.firstinspires.ftc.teamcode.systems.SwerveDrive;

public class Reloca extends Task {
    private SwerveDrive swerveDrive;

    public Reloca(SwerveDrive swerveDrive) {
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
