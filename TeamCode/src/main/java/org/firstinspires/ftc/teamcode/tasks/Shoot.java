package org.firstinspires.ftc.teamcode.tasks;

import org.firstinspires.ftc.teamcode.lioncore.tasks.Task;
import org.firstinspires.ftc.teamcode.systems.Intake;

public class Shoot extends Task {

    private Intake intake;
    private long initTime;

    public Shoot(Intake intake) {
        this.intake = intake;
    }

    @Override
    public void init() {
        this.initTime = System.nanoTime();
        this.intake.setState(Intake.State.Shooting);
    }

    @Override
    public boolean finished() {
        return (System.nanoTime() - initTime) > 1e9;
    }

    @Override
    public void end(boolean i) {
        this.intake.setState(Intake.State.Off);
    }
}
