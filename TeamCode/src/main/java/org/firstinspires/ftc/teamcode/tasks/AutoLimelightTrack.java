package org.firstinspires.ftc.teamcode.tasks;

import org.firstinspires.ftc.teamcode.lioncore.math.types.Position;
import org.firstinspires.ftc.teamcode.lioncore.math.types.Vector2;
import org.firstinspires.ftc.teamcode.lioncore.paths.Line;
import org.firstinspires.ftc.teamcode.lioncore.tasks.Task;
import org.firstinspires.ftc.teamcode.systems.Follower;
import org.firstinspires.ftc.teamcode.systems.Limelight;
import org.firstinspires.ftc.teamcode.systems.SwerveDrive;

public class AutoLimelightTrack extends Task {

    private Follower drivetrain;
    private Limelight limelight;
    private long startTime;

    private boolean foundSolution;
    private boolean startedPathing;
    private Double angle;
    private int pathingLoops;

    public AutoLimelightTrack(Follower drivetrain, Limelight limelight) {
        this.drivetrain = drivetrain;
        this.limelight = limelight;
    }

    @Override
    public void init() {
        this.angle = null;
        this.pathingLoops = 0;
        this.foundSolution = false;
        this.startedPathing = false;
        this.startTime = System.nanoTime();
    }

    @Override
    public void run() {
        if (foundSolution) {
            if (!startedPathing) {
                double heading = angle - SwerveDrive.PinpointCache.position.heading;
                Vector2 position = SwerveDrive.PinpointCache.position.position;
                Vector2 delta = Vector2.polar(1000, Math.toRadians( 90 - heading)).add(position);
                double x = Math.max(Math.min(delta.x(), 3100), -3100);
                double y = -100;

                if (x > 0 && x < 1900) x = 1900;
                if (x < 0 && x > -1900) x = -1900;

                delta.update(x, y);

                this.drivetrain.follow(new Line(
                        new Position(position.x(), position.y(), SwerveDrive.PinpointCache.position.heading),
                        new Position(delta.x(), delta.y(), SwerveDrive.PinpointCache.position.heading)
                ));
                this.startedPathing = true;
            } else {
                this.pathingLoops += 1;
            }
        } else {
            Double angle = limelight.angle();
            if (angle == null) return;
            this.angle = angle;
            this.foundSolution = true;
        }
    }

    @Override
    public boolean finished() {
        return (!foundSolution && System.nanoTime() - startTime > 3e9)
                || (startedPathing && drivetrain.getDistance() < 150) || (pathingLoops > 5 && drivetrain.driver());
    }

    @Override
    public void end(boolean i) {
        if (this.startedPathing) this.drivetrain.stop();
        this.startedPathing = false;
        this.foundSolution = false;
        this.angle = null;
    }
}
