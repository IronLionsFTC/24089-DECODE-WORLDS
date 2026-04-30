package org.firstinspires.ftc.teamcode.tasks;

import org.firstinspires.ftc.teamcode.lioncore.tasks.Task;
import org.firstinspires.ftc.teamcode.systems.Shooter;

public class DownAdjust extends Task {
    public DownAdjust() {}

    @Override
    public void init() {
        Shooter.ShooterPID.targetYFar -= 10;
        Shooter.ShooterPID.targetYClose -= 10;
    }

    @Override
    public boolean finished() {
        return true;
    }
}
