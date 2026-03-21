package org.firstinspires.ftc.teamcode.tasks;

import org.firstinspires.ftc.teamcode.lioncore.tasks.Task;
import org.firstinspires.ftc.teamcode.systems.Limelight;
import org.firstinspires.ftc.teamcode.systems.SwerveDrive;

public class LimelightRelocalise extends Task {

    private SwerveDrive drivetrain;
    private Limelight limelight;

    public LimelightRelocalise(SwerveDrive drivetrain, Limelight limelight) {
        this.drivetrain = drivetrain;
        this.limelight = limelight;
    }

    @Override
    public void init() {
        this.drivetrain.relocaliseTo(limelight.position);
    }

    @Override
    public boolean finished() {
        return true;
    }
}
