package org.firstinspires.ftc.teamcode.tasks;

import org.firstinspires.ftc.teamcode.lioncore.tasks.Task;
import org.firstinspires.ftc.teamcode.systems.Intake;
import org.firstinspires.ftc.teamcode.systems.Shooter;

public class IntakeUntilFull extends Task {

    private final Intake intake;

    public IntakeUntilFull(Intake intake) {
        this.intake = intake;
    }

    @Override
    public void init() {
        this.intake.setState(Intake.State.IntakingEmpty);
    }

    @Override
    public boolean finished() {
        return this.intake.getState() == Intake.State.Off;
    }
}
