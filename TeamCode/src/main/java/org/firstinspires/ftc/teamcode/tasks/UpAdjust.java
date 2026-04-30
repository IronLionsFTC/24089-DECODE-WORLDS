package org.firstinspires.ftc.teamcode.tasks;

import org.firstinspires.ftc.teamcode.lioncore.tasks.Task;
import org.firstinspires.ftc.teamcode.systems.Shooter;

public class UpAdjust extends Task {
    Shooter shooter;
    public UpAdjust(Shooter shooter) {
        this.shooter = shooter;
    }

    @Override
    public void init() {
        shooter.upAdjust();
    }

    @Override
    public boolean finished() {
        return true;
    }
}
