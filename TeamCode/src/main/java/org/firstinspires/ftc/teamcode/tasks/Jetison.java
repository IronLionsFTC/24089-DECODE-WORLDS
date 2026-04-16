package org.firstinspires.ftc.teamcode.tasks;

import org.firstinspires.ftc.teamcode.lioncore.tasks.Task;
import org.firstinspires.ftc.teamcode.systems.Intake;

public class Jetison extends Task {

    private Intake intake;
    private Intake.State prevState;
    long endTime;

    public Jetison(Intake intake) {
        this.intake = intake;
    }

    @Override
    public void init() {
        this.prevState = intake.getState();
        this.intake.setState(Intake.State.Reverse);
        this.endTime = (long) (System.nanoTime() + 2e8);
    }

    @Override
    public boolean finished() {
        return System.nanoTime() > endTime;
    }

    @Override
    public void end(boolean i) {
        this.intake.setState(prevState);
    }
}
