package org.firstinspires.ftc.teamcode.tasks;

import org.firstinspires.ftc.teamcode.lioncore.tasks.Task;
import org.firstinspires.ftc.teamcode.systems.Shooter;

public class RightAdjust extends Task {
    Shooter shooter;
    public RightAdjust(Shooter shooter) {
        this.shooter = shooter;
    }

    @Override
    public void init() {
        shooter.rightAdjust();
    }

    @Override
    public boolean finished() {
        return true;
    }
}
