package org.firstinspires.ftc.teamcode.tasks;

import org.firstinspires.ftc.teamcode.lioncore.tasks.Task;
import org.firstinspires.ftc.teamcode.systems.Shooter;

public class DownAdjust extends Task {
    Shooter shooter;
    public DownAdjust(Shooter shooter) {
        this.shooter = shooter;
    }

    @Override
    public void init() {
        shooter.downAdjust();
    }

    @Override
    public boolean finished() {
        return true;
    }
}
