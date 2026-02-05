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
        public static double P = 0.0005;
        public static double I = 0.0;
        public static double D = 0.0;
        public static double F = 0.15;
    }

    public VelocityFollower() {
        this.velocityController = new PID(
                VelocityPID.P,
                VelocityPID.I,
                VelocityPID.D
        );

        this.targetFieldCentricVelocity = Vector.cartesian(0, 0);
        this.targetRobotCentricVelocity = Vector.cartesian(0, 0);
        this.swerveDrive = new SwerveDrive(new Position(0, 0, 0), false);
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

        this.velocityController.setConstants(
                VelocityPID.P,
                VelocityPID.I,
                VelocityPID.D
        );

        // Translate the field centric movement back into robot centric
        double headingRadians = Math.toRadians(SwerveDrive.PinpointCache.position.heading);
        double c = Math.cos(headingRadians);
        double s = Math.sin(headingRadians);

        targetRobotCentricVelocity.update(
                targetFieldCentricVelocity.x() * c + targetFieldCentricVelocity.y() * s,
                -targetFieldCentricVelocity.x() * s + targetFieldCentricVelocity.y() * c
        );

        double response = velocityController.calculate(SwerveDrive.PinpointCache.velocity.magnitude(), targetRobotCentricVelocity.magnitude());
        if (response != 0) response += VelocityPID.F;
        response = Math.max(0, Math.min(1, response));

        targetRobotCentricVelocity.normalise();
        targetRobotCentricVelocity.multiply_mut(response);

        swerveDrive.setTargetVector(targetRobotCentricVelocity);
        swerveDrive.update(telemetry);

        telemetry.addData("XV", SwerveDrive.PinpointCache.velocity.x());
        telemetry.addData("YV", SwerveDrive.PinpointCache.velocity.y());
        telemetry.addData("FX", targetFieldCentricVelocity.x());
        telemetry.addData("FY", targetFieldCentricVelocity.y());
        telemetry.addData("RX", targetRobotCentricVelocity.x());
        telemetry.addData("RY", targetRobotCentricVelocity.y());
    }

    public void setTargetFieldCentricVelocity(Vector targetFieldCentricVelocity) {
        this.targetFieldCentricVelocity.copy(targetFieldCentricVelocity);
    }

    public void setTargetHeading(double targetHeading) {
        this.swerveDrive.setTargetHeading(targetHeading);
    }
}
