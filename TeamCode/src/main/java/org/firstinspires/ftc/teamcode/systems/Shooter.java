package org.firstinspires.ftc.teamcode.systems;

import static org.firstinspires.ftc.teamcode.projectileMotion.ProjectileMotion.ProjMotConstants.shooterOffset;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Opmodes.Debug.ZeroTurret;
import org.firstinspires.ftc.teamcode.lioncore.hardware.AbsoluteEncoder;
import org.firstinspires.ftc.teamcode.lioncore.hardware.Encoder;
import org.firstinspires.ftc.teamcode.lioncore.hardware.LionCRServo;
import org.firstinspires.ftc.teamcode.lioncore.hardware.LionMotor;
import org.firstinspires.ftc.teamcode.lioncore.hardware.LionServo;
import org.firstinspires.ftc.teamcode.lioncore.math.pid.PID;
import org.firstinspires.ftc.teamcode.lioncore.math.types.Vector2;
import org.firstinspires.ftc.teamcode.lioncore.math.types.Vector3;
import org.firstinspires.ftc.teamcode.lioncore.systems.SystemBase;
import org.firstinspires.ftc.teamcode.lioncore.tasks.TaskOpMode;
import org.firstinspires.ftc.teamcode.parameters.MotorConstants;
import org.firstinspires.ftc.teamcode.parameters.ServoConstants;
import org.firstinspires.ftc.teamcode.projectileMotion.ProjectileMotion;

public class Shooter extends SystemBase {

    public enum State {
        Cruising,
        Shooting
    }

    public State state = State.Cruising;

    // Hardware
    private LionMotor motors;
    private LionServo hoodServo;
    private LionCRServo leftTurretServo;
    private LionCRServo rightTurretServo;
    private Encoder quadrature;

    // Control
    private PID pid;
    private PID turretpid;
    public final Vector3 target;
    public double targetVelocity;
    public double targetHood;

    public Shooter() {
        this.targetVelocity = 0;
        this.targetHood = 0;
        this.target = new Vector3(0, 2000, 800);
    }

    public static final double a = 0.00137115;
    public static final double b = 1.66917;

    // PID constants
    @Config
    public static class ShooterPID {
        public static double P = 0.003;
        public static double I = 0;
        public static double D = 0.00003;
        public static double kS = 0.07;
        public static double kV = 0.00017;

        public static double velocity = 3900;

        // +x = right, +y = forward, +z = up
        public static double targetX = -1000;
        public static double targetY = -2900;
        public static double targetZ = 1600;

        public static boolean flatShot = false;
    }

    @Override
    public void loadHardware(HardwareMap hardwareMap) {
        this.motors = LionMotor.masterSlaves(hardwareMap, MotorConstants.Names.leftShooterMotor, MotorConstants.Names.rightShooterMotor);
        this.hoodServo = LionServo.single(hardwareMap, ServoConstants.Names.hoodServo, ServoConstants.Zero.hood);

        AbsoluteEncoder absolute = new AbsoluteEncoder(hardwareMap, "turretAbsolute");
        this.quadrature = motors.yieldEncoder(1);
        this.quadrature.reverse();

        absolute.read();
        double absolutePosition = ((absolute.position() - ServoConstants.Zero.turret) / 3.3) * 360;
        while (absolutePosition < -180.0) absolutePosition += 360;
        while (absolutePosition >  180.0) absolutePosition -= 360;

        double inTicks = absolutePosition / 360 * 4096;
        quadrature.setPosition(inTicks);

        this.leftTurretServo = new LionCRServo(hardwareMap, ServoConstants.Names.leftTurretServo);
        this.rightTurretServo = new LionCRServo(hardwareMap, ServoConstants.Names.rightTurretServo);
        this.leftTurretServo.setReversed(true);
    }

    @Override
    public void init() {
        this.pid = new PID(
                ShooterPID.P,
                ShooterPID.I,
                ShooterPID.D
        );
        this.turretpid = new PID(
                ZeroTurret.TurretPID.P,
                ZeroTurret.TurretPID.I,
                ZeroTurret.TurretPID.D
        );
        this.motors.setReversed(MotorConstants.Reversed.leftShooterMotor, MotorConstants.Reversed.rightShooterMotor);
        this.motors.setZPB(MotorConstants.ZPB.shooterMotors);
    }

    @Override
    public void update(Telemetry telemetry) {
        this.pid.setConstants(
                ShooterPID.P,
                ShooterPID.I,
                ShooterPID.D
        );

        double current = this.motors.getVelocity(28.0);
        telemetry.addData("Flywheel RPM", current);

        // Solve for position of the turret
        double cosHeading = Math.cos(SwerveDrive.PinpointCache.position.heading);
        double sinHeading = Math.sin(SwerveDrive.PinpointCache.position.heading);

        double rotatedShooterX = shooterOffset.getX() * cosHeading - shooterOffset.getY() * sinHeading;
        double rotatedShooterY = shooterOffset.getX() * sinHeading + shooterOffset.getY() * cosHeading;
        double rotatedShooterZ = shooterOffset.getZ();

        Vector3 shooterPositionInField = new Vector3(
                SwerveDrive.PinpointCache.position.position.x() + rotatedShooterX,
                SwerveDrive.PinpointCache.position.position.y() + rotatedShooterY,
                rotatedShooterZ
        );

        double launchVelocity = ProjectileMotion.calculateAverageSpeedFromLaunchVelocity(rpmToVelocity(current), 0.7);
        Vector3 relativeTarget = target.subtract(shooterPositionInField);

        Vector2 groundPlane = Vector2.cartesian(relativeTarget.getX(), relativeTarget.getY());
        double direction = 180 + (groundPlane.polarDirection() + SwerveDrive.PinpointCache.position.heading);
        while (direction < -180) direction += 360;
        while (direction >  180) direction -= 360;

        Vector2 cartesianTarget = Vector2.cartesian(Math.hypot(relativeTarget.getX(), relativeTarget.getY()), relativeTarget.getZ());
        ProjectileMotion solution = ProjectileMotion.calculate(launchVelocity, cartesianTarget);

        double hoodTarget;
        double tof;
        if (solution.flatPossible && ShooterPID.flatShot) {
            hoodTarget = solution.flat;
            tof = solution.flatTimeOfFlight;
        } else if (solution.arcPossible && !ShooterPID.flatShot) {
            hoodTarget = solution.arc;
            tof = solution.arcTimeOfFlight;
        } else {
            hoodTarget = 75;
            tof = 1;
        }

        this.targetVelocity = groundPlane.magnitude() * 1.3;
        this.targetVelocity = ProjectileMotion.calculateRequiredLaunchVelocity(targetVelocity, tof);
        double response = this.pid.calculate(current, targetVelocity);
        double targetRPM = this.velocityToRPM(targetVelocity);
        double feedforward = ShooterPID.kS * Math.signum(targetRPM) + ShooterPID.kV * targetRPM;
        feedforward *= TaskOpMode.Runtime.voltageCompensation;
        if (targetRPM != 0) response += feedforward;

        if (this.state == State.Cruising) this.motors.setPower(response);
        else {
            if (current < targetVelocity) this.motors.setPower(1);
            else this.motors.setPower(0);
        }
        this.hoodServo.setPosition(calculateHoodAngleForDegrees(hoodTarget));

        this.target.setX(ShooterPID.targetX);
        this.target.setY(ShooterPID.targetY);
        this.target.setZ(ShooterPID.targetZ);

        double quadraturePosition = quadrature.getPosition() / 4096 * 360;
        while (quadraturePosition < -180.0) quadraturePosition += 360;
        while (quadraturePosition >  180.0) quadraturePosition -= 360;
        response = this.turretpid.calculate(quadraturePosition, direction);
        this.leftTurretServo.setPower(response);
        this.rightTurretServo.setPower(response);
    }

    public double getHoodAngleDegrees() {
        return 90 - (this.hoodServo.getPosition() * ServoConstants.Ratios.hoodRatio * ServoConstants.Ratios.hoodAngle + ServoConstants.Ratios.hoodZeroAngle);
    }

    public double calculateHoodAngleForDegrees(double degrees) {
        return ((90 - degrees) - ServoConstants.Ratios.hoodZeroAngle) / (ServoConstants.Ratios.hoodRatio * ServoConstants.Ratios.hoodAngle);
    }

    /**
     * Convert instantaneous RPM into launch velocity
     * @return velocity (mm/s)
     */
    public double rpmToVelocity(double rpm) {
        return (a * rpm + b) * 1000;
    }

    public double velocityToRPM(double velocity) {
        return (velocity / 1000 - b) / a;
    }
}
