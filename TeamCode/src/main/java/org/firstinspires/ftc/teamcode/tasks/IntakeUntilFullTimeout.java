package org.firstinspires.ftc.teamcode.tasks;

import org.firstinspires.ftc.teamcode.lioncore.tasks.Task;
import org.firstinspires.ftc.teamcode.systems.Intake;

public class IntakeUntilFullTimeout extends Task {

    private final Intake intake;
    private long start;
    private long end;
    private final double seconds;

    public IntakeUntilFullTimeout(Intake intake, double seconds) {
        this.intake = intake;
        this.seconds = seconds;
    }

    @Override
    public void init() {
        this.intake.setState(Intake.State.IntakingEmpty);
        this.start = System.nanoTime();
        this.end = start + (long)(this.seconds * 1_000_000_000);
    }

    @Override
    public boolean finished() {
        return this.intake.getState() == Intake.State.Off || System.nanoTime() > end;
    }

    @Override
    public void end(boolean i) {
        this.intake.setState(Intake.State.Off);
    }
}
