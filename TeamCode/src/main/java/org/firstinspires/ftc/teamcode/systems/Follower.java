package org.firstinspires.ftc.teamcode.systems;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.lioncore.math.pid.PID;
import org.firstinspires.ftc.teamcode.lioncore.math.types.Path;
import org.firstinspires.ftc.teamcode.lioncore.math.types.Position;
import org.firstinspires.ftc.teamcode.lioncore.math.types.Vector;
import org.firstinspires.ftc.teamcode.lioncore.systems.SystemBase;
import org.firstinspires.ftc.teamcode.lioncore.tasks.TaskOpMode;

public class Follower extends SystemBase {

    private final VelocityFollower drivetrain;
    private Path path;

    // Mutable vectors
    private final Position closest;
    private final Position discreteStep;
    private final Vector tangent;
    private final Vector normal;
    private final Vector corrective;
    private final Vector targetVelocity;

    // Speed
    private double currentSpeed;

    @Config
    public static class FollowerConstants {
        public static double translationP = 0.001;
        public static double maxSpeed = 2000;
        public static double acceleration = 1000;
        public static double deceleration = 5000;
    }

    public Follower() {
        this.drivetrain = new VelocityFollower();
        this.closest = new Position(0, 0, 0);
        this.discreteStep = new Position(0, 0, 0);
        this.tangent = Vector.cartesian(0, 0);
        this.normal = Vector.cartesian(0, 0);
        this.corrective = Vector.cartesian(0, 0);
        this.targetVelocity = Vector.cartesian(0, 0);
        this.currentSpeed = 0;
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
    public void update(Telemetry telemetry) {
        if (path == null) {
            this.currentSpeed = 0;
            this.targetVelocity.update(0, 0);
            this.drivetrain.setTargetFieldCentricVelocity(this.targetVelocity);
            this.drivetrain.update(telemetry);
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

        // Compute trapezoidal velocity profile along the path
        double distanceRemaining = path.distanceRemaining();
        double currentSpeed = SwerveDrive.PinpointCache.velocity.magnitude();

        // max velocity and acceleration constants
        double vmax = FollowerConstants.maxSpeed;
        double stoppingDistance = Math.pow(vmax, 2) / (2 * FollowerConstants.deceleration);

        if (distanceRemaining > stoppingDistance) {
            currentSpeed += TaskOpMode.Runtime.deltaTime * FollowerConstants.acceleration;
            if (currentSpeed > vmax) currentSpeed = vmax;
        } else {
            // Decelerate smoothly to stop at path end
            currentSpeed -= TaskOpMode.Runtime.deltaTime * FollowerConstants.deceleration;
            if (currentSpeed < 400) currentSpeed = 400;
        }

        this.tangent.multiply_mut(currentSpeed);

        // Compute error, perpendicular to tangential motion
        this.corrective.update(-tangent.y(), tangent.x());
        double lateralError = normal.dot(corrective);
        this.corrective.normalise();
        this.corrective.multiply_mut(lateralError * FollowerConstants.translationP);

        // Combine target velocities
        this.tangent.add_into(this.corrective, this.targetVelocity);

        // Apply into the velocityFollower
        this.drivetrain.setTargetFieldCentricVelocity(this.targetVelocity);
        this.drivetrain.setTargetHeading(this.closest.heading);
        this.drivetrain.update(telemetry);
    }

    public double getDistance() {
        return this.path.distanceRemaining();
    }

    public void stop() {
        this.path = null;
    }

    public void follow(Path path) {
        this.currentSpeed = 0;
        this.path = path;
    }
}
