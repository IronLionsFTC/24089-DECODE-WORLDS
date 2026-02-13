package org.firstinspires.ftc.teamcode.systems;

import static org.firstinspires.ftc.teamcode.projectileMotion.ProjectileMotion.ProjMotConstants.shooterOffset;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;
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

    // Hardware
    private LionMotor motors;
    private LionServo hoodServo;

    // Control
    private PID pid;
    public final Vector3 target;
    public double targetRPM = 0;
    public double targetHood = 0 ;

    public Shooter() {
        this.targetRPM = 0;
        this.targetHood = 0;
        this.target = new Vector3(0, 2000, 800);
    }

    // PID constants
    @Config
    public static class ShooterPID {
        public static double P = 0.003;
        public static double I = 0;
        public static double D = 0.00003;
        public static double kS = 0.07;
        public static double kV = 0.00017;

        // +x = right, +y = forward, +z = up
        public static double targetX = 0;
        public static double targetY = 0;
        public static double targetZ = 0;

        public static boolean flatShot = false;
    }

    @Override
    public void loadHardware(HardwareMap hardwareMap) {

        this.motors = LionMotor.masterSlaves(hardwareMap, MotorConstants.Names.leftShooterMotor, MotorConstants.Names.rightShooterMotor);
        this.hoodServo = LionServo.single(hardwareMap, ServoConstants.Names.hoodServo, ServoConstants.Zero.hood);
    }

    @Override
    public void init() {
        this.pid = new PID(
                ShooterPID.P,
                ShooterPID.I,
                ShooterPID.D
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

        double response = this.pid.calculate(current, targetRPM);

        double feedforward = ShooterPID.kS * Math.signum(targetRPM) + ShooterPID.kV * targetRPM;
        feedforward *= TaskOpMode.Runtime.voltageCompensation;
        if (targetRPM != 0) response += feedforward;

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

        double launchVelocity = rpmToVelocity(current);
        Vector3 relativeTarget = target.subtract(shooterPositionInField);
        Vector2 cartesianTarget = Vector2.cartesian(Math.hypot(relativeTarget.getX(), relativeTarget.getY()), relativeTarget.getZ());
        ProjectileMotion solution = ProjectileMotion.calculate(launchVelocity, cartesianTarget);

        this.motors.setPower(response);

        double hoodTarget;
        if (solution.flatPossible && ShooterPID.flatShot) {
            hoodTarget = solution.flat;
        } else if (solution.arcPossible && !ShooterPID.flatShot) {
            hoodTarget = solution.arc;
        } else {
            hoodTarget = 75;
        }

        this.hoodServo.setPosition(calculateHoodAngleForDegrees(hoodTarget));

        this.target.setX(ShooterPID.targetX);
        this.target.setY(ShooterPID.targetY);
        this.target.setZ(ShooterPID.targetZ);
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
        return (rpm * 0.00125699 + 2.15515) * 1000;
    }
}
