package org.firstinspires.ftc.teamcode.systems;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.lioncore.math.types.Path;
import org.firstinspires.ftc.teamcode.lioncore.math.types.Position;
import org.firstinspires.ftc.teamcode.lioncore.math.types.Vector2;
import org.firstinspires.ftc.teamcode.lioncore.math.types.Vector3;
import org.firstinspires.ftc.teamcode.lioncore.systems.SystemBase;

import java.util.function.DoubleSupplier;

public class Follower extends SystemBase {

    private final VelocityFollower drivetrain;
    private Path path;

    // Mutable vectors
    private final Position closest;
    private final Position discreteStep;
    private final Vector2 tangent;
    private final Vector2 normal;
    private final Vector2 corrective;
    private final Vector2 targetVelocity;

    // Speed
    private double targetSpeed;

    @Config
    public static class FollowerConstants {
        public static double translationP = 0.003;
        public static double maxSpeed = 1400;
    }

    public Follower(double x, double y, double h) {
        this.drivetrain = new VelocityFollower(x, y, h);
        this.closest = new Position(0, 0, h);
        this.discreteStep = new Position(0, 0, h);
        this.tangent = Vector2.cartesian(0, 0);
        this.normal = Vector2.cartesian(0, 0);
        this.corrective = Vector2.cartesian(0, 0);
        this.targetVelocity = Vector2.cartesian(0, 0);
        this.targetSpeed = 0;
    }

    public Follower(double x, double y, double h, DoubleSupplier heading) {
        this.drivetrain = new VelocityFollower(x, y, h, heading);
        this.closest = new Position(0, 0, h);
        this.discreteStep = new Position(0, 0, h);
        this.tangent = Vector2.cartesian(0, 0);
        this.normal = Vector2.cartesian(0, 0);
        this.corrective = Vector2.cartesian(0, 0);
        this.targetVelocity = Vector2.cartesian(0, 0);
        this.targetSpeed = 0;
    }

    @Override
    public void loadHardware(HardwareMap hardwareMap) {
        this.drivetrain.loadHardware(hardwareMap);
    }

    @Override
    public void init() {
        this.drivetrain.init();
    }

    @Override
    public void update(Telemetry telemetry, boolean useTelemetry) {
        if (path == null) {
            this.targetSpeed = 0;
            this.targetVelocity.update(0, 0);
            this.drivetrain.setTargetFieldCentricVelocity(this.targetVelocity);
            this.drivetrain.update(telemetry, useTelemetry);
            return;
        }

        // Compute current position and next position for discrete derivative evaluation
        double k = this.path.getClosestK();
        this.path.getTarget(k, this.closest);
        this.path.getTarget(k + 0.05, this.discreteStep);

        // Compute tangent and normal to the path at the current point
        this.discreteStep.position.sub_into(this.closest.position, this.tangent);
        this.closest.position.sub_into(SwerveDrive.PinpointCache.position.position, this.normal);
        this.tangent.normalise();

        // max velocity and acceleration constants
        targetSpeed = FollowerConstants.maxSpeed;
        this.tangent.multiply_mut(targetSpeed);

        // Compute error, perpendicular to tangential motion
        this.corrective.update(-tangent.y(), tangent.x());
        double lateralError = normal.dot(corrective);
        this.corrective.normalise();
        this.corrective.multiply_mut(lateralError * FollowerConstants.translationP);

        // Combine target velocities
        this.tangent.add_into(this.corrective, this.targetVelocity);

        if (path.distanceRemaining() > 220) {
            // Apply into the velocityFollower
            this.drivetrain.state = VelocityFollower.State.Velocity;
            this.drivetrain.setTargetFieldCentricVelocity(this.targetVelocity);
            this.drivetrain.setTargetHeading(this.closest.heading);
        } else {
            // Holdpoint
            this.drivetrain.state = VelocityFollower.State.Holdpoint;
            this.path.getTarget(1, this.closest);
            this.drivetrain.setHoldpoint(closest);
        }

        this.drivetrain.update(telemetry, useTelemetry);
    }

    public double getDistance() {
        return this.path.distanceRemaining();
    }

    public void stop() {
        this.path = null;
    }

    public void follow(Path path) {
        this.targetSpeed = 0;
        this.path = path;
    }

    public void setPosition(Position position) {
        this.drivetrain.setPosition(position);
    }

    public void setDriveInput(Vector2 driveInput) {
        this.drivetrain.setDriveInput(driveInput);
    }

    public void setXPattern(boolean xPattern) {
        this.drivetrain.setxPattern(xPattern);
    }

    public void relocalise() {
        this.drivetrain.relocalise();
    }

    public void relocaliseTo(Position position) {
        this.drivetrain.relocaliseTo(position);
    }

    public void setHeading(double heading) {
        this.drivetrain.forceHeading(heading);
    }

    public boolean driver() {
        return this.drivetrain.state == VelocityFollower.State.Driver;
    }
}
