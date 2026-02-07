package org.firstinspires.ftc.teamcode.systems;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.lioncore.math.pid.PID;
import org.firstinspires.ftc.teamcode.lioncore.math.types.Position;
import org.firstinspires.ftc.teamcode.lioncore.math.types.Vector;
import org.firstinspires.ftc.teamcode.lioncore.systems.SystemBase;
import org.firstinspires.ftc.teamcode.lioncore.tasks.TaskOpMode;

public class VelocityFollower extends SystemBase {

    private final SwerveDrive swerveDrive;
    private final PID velocityController;
    private final PID holdpointController;
    private final Vector targetFieldCentricVelocity;
    private final Vector targetRobotCentricVelocity;
    private final Position holdpoint;

    public State state;

    public enum State {
        Velocity,
        Holdpoint
    }

    @Config
    public static class VelocityPID {
        public static double P = 0.0005;
        public static double I = 0.0;
        public static double D = 0.0;
        public static double kS = 0.1;
        public static double kV = 0.0004;
    }

    @Config
    public static class HoldpointPID {
        public static double P = 0.002;
        public static double I = 0.0;
        public static double D = 0.0008;
        public static double kS = 0.2;
        public static double kV = 0.0;
    }

    public VelocityFollower() {
        this.velocityController = new PID(
                VelocityPID.P,
                VelocityPID.I,
                VelocityPID.D
        );

        this.holdpointController = new PID(
                HoldpointPID.P,
                HoldpointPID.I,
                HoldpointPID.D
        );

        this.targetFieldCentricVelocity = Vector.cartesian(0, 0);
        this.targetRobotCentricVelocity = Vector.cartesian(0, 0);
        this.swerveDrive = new SwerveDrive(new Position(0, 0, 0), true);
        this.holdpoint = new Position(0, 0, 0);
        this.state = State.Velocity;
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

        double headingRadians;
        double c;
        double s;
        double response;
        double feedforward;

        switch (this.state) {

            case Velocity:
                this.velocityController.setConstants(
                        VelocityPID.P,
                        VelocityPID.I,
                        VelocityPID.D
                );

                // Translate the field centric movement back into robot centric
                headingRadians = Math.toRadians(SwerveDrive.PinpointCache.position.heading);
                c = Math.cos(headingRadians);
                s = Math.sin(headingRadians);

                targetRobotCentricVelocity.update(
                        targetFieldCentricVelocity.x() * c + targetFieldCentricVelocity.y() * s,
                        -targetFieldCentricVelocity.x() * s + targetFieldCentricVelocity.y() * c
                );

                double targetVelocity = targetRobotCentricVelocity.magnitude();
                response = velocityController.calculate(SwerveDrive.PinpointCache.velocity.magnitude(), targetVelocity);
                feedforward = VelocityPID.kS * Math.signum(targetVelocity) + VelocityPID.kV * targetVelocity;
                response = Math.max(0, Math.min(1, response + feedforward * TaskOpMode.Runtime.voltageCompensation));

                targetRobotCentricVelocity.normalise();
                targetRobotCentricVelocity.multiply_mut(response);
                break;

            case Holdpoint:
                this.holdpointController.setConstants(
                        HoldpointPID.P,
                        HoldpointPID.I,
                        HoldpointPID.D
                );

                this.holdpoint.position.sub_into(SwerveDrive.PinpointCache.position.position, this.targetFieldCentricVelocity);
                double distance = this.targetFieldCentricVelocity.magnitude();
                this.targetFieldCentricVelocity.normalise();

                response = holdpointController.calculate(-distance, 0);
                feedforward = HoldpointPID.kS;
                if (Math.abs(response) > 0.05) response = Math.max(0, Math.min(1, response + feedforward * TaskOpMode.Runtime.voltageCompensation));

                // Translate the field centric movement back into robot centric
                headingRadians = Math.toRadians(SwerveDrive.PinpointCache.position.heading);
                c = Math.cos(headingRadians);
                s = Math.sin(headingRadians);

                targetRobotCentricVelocity.update(
                        targetFieldCentricVelocity.x() * c + targetFieldCentricVelocity.y() * s,
                        -targetFieldCentricVelocity.x() * s + targetFieldCentricVelocity.y() * c
                );

                targetRobotCentricVelocity.normalise();
                targetRobotCentricVelocity.multiply_mut(response);
                this.swerveDrive.setTargetHeading(holdpoint.heading);
                break;
        }

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

    public void setHoldpoint(Position holdpoint) {
        this.holdpoint.position.copy(holdpoint.position);
        this.holdpoint.heading = holdpoint.heading;
    }
}
