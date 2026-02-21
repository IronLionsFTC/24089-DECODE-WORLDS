package org.firstinspires.ftc.teamcode.systems;

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
import org.firstinspires.ftc.teamcode.parameters.Zeroing;

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

        public static double targetXFar = -4000;
        public static double targetYFar = 0;
        public static double targetZFar = 0;

        public static double targetXClose = 0;
        public static double targetYClose = -5000;
        public static double targetZClose = 0;

        public static double velocityOverride = 3000;
        public static double servoPositionOverride = 0;
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

        double rotatedShooterX = Zeroing.ProjMotConstants.shooterOffset.getX() * cosHeading - Zeroing.ProjMotConstants.shooterOffset.getY() * sinHeading;
        double rotatedShooterY = Zeroing.ProjMotConstants.shooterOffset.getX() * sinHeading + Zeroing.ProjMotConstants.shooterOffset.getY() * cosHeading;
        double rotatedShooterZ = Zeroing.ProjMotConstants.shooterOffset.getZ();

        Vector3 shooterPositionInField = new Vector3(
                SwerveDrive.PinpointCache.position.position.x() + rotatedShooterX,
                SwerveDrive.PinpointCache.position.position.y() + rotatedShooterY,
                rotatedShooterZ
        );

        Vector3 relativeTarget = target.subtract(shooterPositionInField);
        Vector2 groundPlane = Vector2.cartesian(relativeTarget.getX(), relativeTarget.getY());
        Vector2 cartesianTarget = Vector2.cartesian(Math.hypot(relativeTarget.getX(), relativeTarget.getY()), relativeTarget.getZ());
        double distance = SwerveDrive.PinpointCache.position.position.sub(
                Vector2.cartesian(ShooterPID.targetXFar, ShooterPID.targetYFar)
        ).magnitude();
        boolean far = distance > 2500;

        double direction = 180 + (groundPlane.polarDirection() + SwerveDrive.PinpointCache.position.heading);
        while (direction < -180) direction += 360;
        while (direction >  180) direction -= 360;

        double angle = ShooterPID.servoPositionOverride;
        double hoodAngle = angle;
        double servoPosition = calculateHoodAngleForDegrees(hoodAngle);

        this.targetVelocity = ShooterPID.velocityOverride;
        double targetRPM = targetVelocity;//this.velocityToRPM(targetVelocity);
        double response = this.pid.calculate(current, targetRPM);
        double feedforward = ShooterPID.kS * Math.signum(targetRPM) + ShooterPID.kV * targetRPM;
        feedforward *= TaskOpMode.Runtime.voltageCompensation;
        if (targetRPM != 0) response += feedforward;

        if (this.state == State.Cruising) this.motors.setPower(response);
        else {
            if (rpmToVelocity(current) < targetVelocity * 1.01) this.motors.setPower(1);
            else this.motors.setPower(response * 0.95);
        }
        if (!Double.isNaN(angle)) this.hoodServo.setPosition(servoPosition);

        if (far) {
            this.target.setX(ShooterPID.targetXFar);
            this.target.setY(ShooterPID.targetYFar);
            this.target.setZ(ShooterPID.targetZFar);
        } else {
            this.target.setX(ShooterPID.targetXClose);
            this.target.setY(ShooterPID.targetYClose);
            this.target.setZ(ShooterPID.targetZClose);
        }

        double quadraturePosition = quadrature.getPosition() / 4096 * 360;
        response = this.turretpid.calculate(quadraturePosition, direction);
        if (Math.abs(quadraturePosition - direction) > 1) {
            response += ZeroTurret.TurretPID.kS * Math.signum(direction - quadraturePosition);
        }
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
