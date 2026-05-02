package org.firstinspires.ftc.teamcode.tasks;

import org.firstinspires.ftc.teamcode.lioncore.tasks.Task;
import org.firstinspires.ftc.teamcode.systems.Shooter;

public class LeftAdjust extends Task {
    Shooter shooter;
    public LeftAdjust(Shooter shooter) {
        this.shooter = shooter;
    }

    @Override
    public void init() {
        shooter.leftAdjust();
    }

    @Override
    public boolean finished() {
        return true;
    }
}
