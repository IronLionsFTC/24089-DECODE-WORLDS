package org.firstinspires.ftc.teamcode.tasks;

import org.firstinspires.ftc.teamcode.lioncore.tasks.Task;
import org.firstinspires.ftc.teamcode.projectileMotion.ProjectileMotion;
import org.firstinspires.ftc.teamcode.systems.Intake;
import org.firstinspires.ftc.teamcode.systems.Shooter;

public class Shoot extends Task {

    private final Intake intake;
    private final Shooter shooter;
    private long initTime;
    private long endTime;

    public Shoot(Intake intake, Shooter shooter) {
        this.intake = intake;
        this.shooter = shooter;
    }

    @Override
    public void init() {
        this.initTime = System.nanoTime();
        this.intake.setState(Intake.State.Shooting);
        this.shooter.state = Shooter.State.Shooting;

        if (ProjectileMotion.far()) {
            this.endTime = (long) (this.initTime + 1e9);
        } else {
            this.endTime = (long) (this.initTime + 6e8);
        }
    }

    @Override
    public boolean finished() {
        return (System.nanoTime() > endTime);
    }

    @Override
    public void end(boolean i) {
        this.intake.setState(Intake.State.Off);
        this.shooter.state = Shooter.State.Cruising;
    }
}
