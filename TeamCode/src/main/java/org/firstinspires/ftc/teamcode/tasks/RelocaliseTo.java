package org.firstinspires.ftc.teamcode.tasks;

import org.firstinspires.ftc.teamcode.lioncore.math.types.Position;
import org.firstinspires.ftc.teamcode.lioncore.tasks.Task;
import org.firstinspires.ftc.teamcode.systems.SwerveDrive;

public class RelocaliseTo extends Task {
    private SwerveDrive swerveDrive;
    private Position position;

    public RelocaliseTo(SwerveDrive swerveDrive, Position position) {
        this.swerveDrive = swerveDrive;
        this.position = position;
    }

    @Override
    public void init() {
        this.swerveDrive.relocaliseTo(position);
    }

    @Override
    public boolean finished() {
        return true;
    }
}
