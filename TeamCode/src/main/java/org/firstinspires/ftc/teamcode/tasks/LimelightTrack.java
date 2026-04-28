package org.firstinspires.ftc.teamcode.tasks;

import org.firstinspires.ftc.teamcode.lioncore.math.types.Position;
import org.firstinspires.ftc.teamcode.lioncore.math.types.Vector2;
import org.firstinspires.ftc.teamcode.lioncore.paths.Line;
import org.firstinspires.ftc.teamcode.lioncore.tasks.Task;
import org.firstinspires.ftc.teamcode.systems.Follower;
import org.firstinspires.ftc.teamcode.systems.Limelight;
import org.firstinspires.ftc.teamcode.systems.SwerveDrive;

import java.util.Vector;

public class LimelightTrack extends Task {

    private Follower drivetrain;
    private Limelight limelight;
    private long startTime;

    private boolean foundSolution = false;
    private boolean startedPathing = false;
    private double angle = 0;

    public LimelightTrack(Follower drivetrain, Limelight limelight) {
        this.drivetrain = drivetrain;
        this.limelight = limelight;
    }

    @Override
    public void init() {
        this.startTime = System.nanoTime();
    }

    @Override
    public void run() {
        if (foundSolution) {
            if (!startedPathing) {
                double heading = angle - SwerveDrive.PinpointCache.position.heading;
                Vector2 position = SwerveDrive.PinpointCache.position.position;
                Vector2 delta = Vector2.polar(800, Math.toRadians( 90 - heading)).add(position);
                this.drivetrain.follow(new Line(
                        new Position(position.x(), position.y(), SwerveDrive.PinpointCache.position.heading),
                        new Position(delta.x(), delta.y(), SwerveDrive.PinpointCache.position.heading)
                ));
                this.startedPathing = true;
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
                || (startedPathing && drivetrain.getDistance() < 150 || drivetrain.driver());
    }
}
