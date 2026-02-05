package org.firstinspires.ftc.teamcode.systems;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.lioncore.math.pid.PID;
import org.firstinspires.ftc.teamcode.lioncore.math.types.Position;
import org.firstinspires.ftc.teamcode.lioncore.math.types.Vector;
import org.firstinspires.ftc.teamcode.lioncore.systems.SystemBase;

public class VelocityFollower extends SystemBase {

    private final SwerveDrive swerveDrive;
    private final PID velocityController;

    private final Vector targetFieldCentricVelocity;
    private final Vector targetRobotCentricVelocity;

    @Config
    public static class VelocityPID {
        public static double P = 0.0;
        public static double I = 0.0;
        public static double D = 0.0;
    }

    @Config
    public static class FollowerHeadingPID {
        public static double P = 0.0;
        public static double I = 0.0;
        public static double D = 0.0;
    }

    public VelocityFollower() {
        this.velocityController = new PID(
                VelocityPID.P,
                VelocityPID.I,
                VelocityPID.D
        );

        this.targetFieldCentricVelocity = Vector.cartesian(0, 0);
        this.targetRobotCentricVelocity = Vector.cartesian(0, 0);
        this.swerveDrive = new SwerveDrive(new Position(0, 0, 0));
    }

    @Override
    public void loadHardware(HardwareMap hardwareMap) {
        this.swerveDrive.loadHardware(hardwareMap);
    }

    @Override
    public void init() {
        this.swerveDrive.init();
    }

    @Override
    public void update(Telemetry telemetry) {

        // Translate the field centric movement back into robot centric
        double headingRadians = Math.toRadians(SwerveDrive.PinpointCache.position.heading);
        double c = Math.cos(headingRadians);
        double s = Math.sin(headingRadians);

        targetRobotCentricVelocity.update(
                targetFieldCentricVelocity.x() * c + targetFieldCentricVelocity.y() * s,
                -targetFieldCentricVelocity.x() * s + targetFieldCentricVelocity.y() * c
        );

        double response = velocityController.calculate(SwerveDrive.PinpointCache.velocity.magnitude(), targetRobotCentricVelocity.magnitude());
        targetRobotCentricVelocity.normalise();
        targetRobotCentricVelocity.multiply_mut(Math.max(-1, Math.min(1, response)));

        swerveDrive.setTargetVector(targetRobotCentricVelocity);
        swerveDrive.update(telemetry);
    }

    public void setTargetFieldCentricVelocity(Vector targetFieldCentricVelocity) {
        this.targetFieldCentricVelocity.copy(targetFieldCentricVelocity);
    }

    public void setTargetHeading(double targetHeading) {
        this.swerveDrive.setTargetHeading(targetHeading);
    }
}
