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
    private boolean abort;

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
        this.abort = false;
    }

    @Override
    public void run() {
        Follower.FollowerConstants.maxSpeed = 1100;
        if (foundSolution) {
            if (!startedPathing) {
                double heading = angle - SwerveDrive.PinpointCache.position.heading;
                Vector2 position = SwerveDrive.PinpointCache.position.position;
                Vector2 delta = Vector2.polar(1000, Math.toRadians( 90 - heading)).add(position);

                double h = SwerveDrive.PinpointCache.position.heading;
                double y;
                if (h > 90 || h < -90) y = -100;
                else y = 3400;

                double x = Math.max(Math.min(delta.x(), 3300), -3300);

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

                if (this.drivetrain.getDistance() < 300 && SwerveDrive.PinpointCache.velocity.magnitude() < 100) {
                    this.abort = true;
                }
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
                || abort
                || (startedPathing && drivetrain.getDistance() < 150) || (pathingLoops > 5 && drivetrain.driver());
    }

    @Override
    public void end(boolean i) {
        Follower.FollowerConstants.maxSpeed = 1300;
        if (this.startedPathing) this.drivetrain.stop();
        this.startedPathing = false;
        this.foundSolution = false;
        this.angle = null;
        this.abort = false;
    }
}
