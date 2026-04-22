package org.firstinspires.ftc.teamcode.tasks;

import org.firstinspires.ftc.teamcode.lioncore.tasks.Task;
import org.firstinspires.ftc.teamcode.systems.Shooter;

public class ToggleSOTM extends Task {
    public ToggleSOTM() {}
    @Override
    public void init() {
        Shooter.ShooterPID.useConvergence = !Shooter.ShooterPID.useConvergence;
    }

    @Override
    public boolean finished() {
        return true;
    }
}
