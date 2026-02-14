package org.firstinspires.ftc.teamcode.tasks;

import org.firstinspires.ftc.teamcode.lioncore.tasks.Task;
import org.firstinspires.ftc.teamcode.systems.Intake;
import org.firstinspires.ftc.teamcode.systems.Shooter;

public class Shoot extends Task {

    private final Intake intake;
    private final Shooter shooter;
    private long initTime;

    public Shoot(Intake intake, Shooter shooter) {
        this.intake = intake;
        this.shooter = shooter;
    }

    @Override
    public void init() {
        this.initTime = System.nanoTime();
        this.intake.setState(Intake.State.Shooting);
        this.shooter.state = Shooter.State.Shooting;
    }

    @Override
    public boolean finished() {
        return (System.nanoTime() - initTime) > 1e9;
    }

    @Override
    public void end(boolean i) {
        this.intake.setState(Intake.State.Off);
        this.shooter.state = Shooter.State.Cruising;
    }
}
