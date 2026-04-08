package org.firstinspires.ftc.teamcode.tasks;

import org.firstinspires.ftc.teamcode.lioncore.tasks.Task;
import org.firstinspires.ftc.teamcode.systems.Limelight;
import org.firstinspires.ftc.teamcode.systems.SwerveDrive;

public class LimelightRelocalise extends Task {

    private SwerveDrive drivetrain;
    private Limelight limelight;
    private long startTime;

    private boolean foundSolution = false;

    public LimelightRelocalise(SwerveDrive drivetrain, Limelight limelight) {
        this.drivetrain = drivetrain;
        this.limelight = limelight;
    }

    @Override
    public void init() {
        this.limelight.start();
        this.startTime = System.nanoTime();
    }

    @Override
    public void run() {
        if (this.limelight.isValid) {
            this.drivetrain.relocaliseTo(limelight.position);
            this.foundSolution = true;
        }
    }

    @Override
    public boolean finished() {
        return foundSolution || System.nanoTime() - startTime > 1e9;
    }

    @Override
    public void end(boolean i) {
        this.limelight.stop();
    }
}
