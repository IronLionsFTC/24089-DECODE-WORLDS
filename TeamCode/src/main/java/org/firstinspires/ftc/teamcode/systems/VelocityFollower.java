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
    private final PID velocityControllerX;
    private final PID velocityControllerY;

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
        this.velocityControllerX = new PID(
                VelocityPID.P,
                VelocityPID.I,
                VelocityPID.D
        );

        this.velocityControllerY = new PID(
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

        this.velocityControllerX.setConstants(
                VelocityPID.P,
                VelocityPID.I,
                VelocityPID.D
        );

        this.velocityControllerY.setConstants(
                VelocityPID.P,
                VelocityPID.I,
                VelocityPID.D
        );

        double headingRadians = Math.toRadians(SwerveDrive.PinpointCache.position.heading);
        double c = Math.cos(headingRadians);
        double s = Math.sin(headingRadians);

        double targetRx = targetFieldCentricVelocity.x() * c + targetFieldCentricVelocity.y() * s;
        double targetRy = -targetFieldCentricVelocity.x() * s + targetFieldCentricVelocity.y() * c;

        if (Math.abs(targetRx) < 5) targetRx = 0;
        if (Math.abs(targetRy) < 5) targetRy = 0;

        double currentVx = SwerveDrive.PinpointCache.velocity.x();
        double currentVy = SwerveDrive.PinpointCache.velocity.y();

        double pidVx = velocityControllerX.calculate(currentVx, targetRx);
        double pidVy = velocityControllerY.calculate(currentVy, targetRy);

        pidVx = Math.max(-1, Math.min(1, pidVx));
        pidVy = Math.max(-1, Math.min(1, pidVy));

        Vector robotVelocityCommand = Vector.cartesian(pidVx, pidVy);

        swerveDrive.setTargetVector(robotVelocityCommand);
        swerveDrive.update(telemetry);

        telemetry.addData("Current VX", currentVx);
        telemetry.addData("Current VY", currentVy);
        telemetry.addData("Target FX", targetFieldCentricVelocity.x());
        telemetry.addData("Target FY", targetFieldCentricVelocity.y());
        telemetry.addData("Command RX", pidVx);
        telemetry.addData("Command RY", pidVy);
    }

    public void setTargetFieldCentricVelocity(Vector targetFieldCentricVelocity) {
        this.targetFieldCentricVelocity.copy(targetFieldCentricVelocity);
    }

    public void setTargetHeading(double targetHeading) {
        this.swerveDrive.setTargetHeading(targetHeading);
    }
}
