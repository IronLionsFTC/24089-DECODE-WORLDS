package org.firstinspires.ftc.teamcode.tasks;

import org.firstinspires.ftc.teamcode.lioncore.tasks.Task;
import org.firstinspires.ftc.teamcode.systems.Follower;
import org.firstinspires.ftc.teamcode.systems.Limelight;

public class LimelightRelocalise extends Task {

    private Follower drivetrain;
    private Limelight limelight;
    private long startTime;

    private boolean foundSolution = false;

    public LimelightRelocalise(Follower drivetrain, Limelight limelight) {
        this.drivetrain = drivetrain;
        this.limelight = limelight;
    }

    @Override
    public void init() {
        this.startTime = System.nanoTime();
    }

    @Override
    public void run() {
        if (this.limelight.isValid && !this.foundSolution) {
            this.drivetrain.relocaliseTo(limelight.position);
            this.foundSolution = true;
        }
    }

    @Override
    public boolean finished() {
        return foundSolution || System.nanoTime() - startTime > 3e9;
    }
}
