package org.firstinspires.ftc.teamcode.tasks;

import org.firstinspires.ftc.teamcode.lioncore.tasks.Task;
import org.firstinspires.ftc.teamcode.systems.Intake;

import java.util.Objects;

public class TeleopIntake extends Task {

    private Intake intake;

    public TeleopIntake(Intake intake) {
        this.intake = intake;
    }

    @Override
    public void init() {
        if (Objects.requireNonNull(this.intake.getState()) == Intake.State.Off) {
            this.intake.setState(Intake.State.IntakingEmpty);
        } else {
            this.intake.setState(Intake.State.Off);
        }
    }

    @Override
    public boolean finished() {
        return true;
    }
}
