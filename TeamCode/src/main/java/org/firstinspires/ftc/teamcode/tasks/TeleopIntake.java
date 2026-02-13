package org.firstinspires.ftc.teamcode.tasks;

import org.firstinspires.ftc.teamcode.lioncore.tasks.Task;
import org.firstinspires.ftc.teamcode.systems.Intake;

public class TeleopIntake extends Task {

    private Intake intake;

    public TeleopIntake(Intake intake) {
        this.intake = intake;
    }

    @Override
    public void init() {

    }
}
