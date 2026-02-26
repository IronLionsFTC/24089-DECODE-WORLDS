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
import org.firstinspires.ftc.teamcode.lioncore.math.KalmanFilter;
import org.firstinspires.ftc.teamcode.lioncore.math.pid.PID;
import org.firstinspires.ftc.teamcode.lioncore.math.types.Vector3;
import org.firstinspires.ftc.teamcode.lioncore.system.ConstantsStorage;
import org.firstinspires.ftc.teamcode.lioncore.systems.SystemBase;
import org.firstinspires.ftc.teamcode.lioncore.tasks.TaskOpMode;
import org.firstinspires.ftc.teamcode.parameters.MotorConstants;
import org.firstinspires.ftc.teamcode.parameters.ServoConstants;
import org.firstinspires.ftc.teamcode.projectileMotion.ProjectileMotion;
import org.firstinspires.ftc.teamcode.projectileMotion.Regressions;

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
    private KalmanFilter rpmFilter;
    private PID turretpid;
    public final Vector3 target;
    public double targetVelocity;
    public double targetHood;

    public Shooter() {
        this.targetVelocity = 0;
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

        public static double targetXFar = -3900;
        public static double targetYFar = 200;
        public static double targetZFar = 800;
        public static double targetXClose = 0;
        public static double targetYClose = -5000;
        public static double targetZClose = 0;

        public static double kalmanQ = 140.0;
        public static double kalmanR = 42.0;
        public static boolean useConvergence = false;
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

        this.rpmFilter = new KalmanFilter(ShooterPID.kalmanQ, ShooterPID.kalmanR, 0.0);
        this.motors.setReversed(MotorConstants.Reversed.leftShooterMotor, MotorConstants.Reversed.rightShooterMotor);
        this.motors.setZPB(MotorConstants.ZPB.shooterMotors);

        ServoConstants.Zero.turret = ConstantsStorage.get("turretZeroVoltage", ServoConstants.Zero.turret);
    }

    @Override
    public void update(Telemetry telemetry, boolean useTelemetry) {
        this.pid.setConstants(
                ShooterPID.P,
                ShooterPID.I,
                ShooterPID.D
        );

        this.turretpid.setConstants(
                ZeroTurret.TurretPID.P,
                ZeroTurret.TurretPID.I,
                ZeroTurret.TurretPID.D
        );

        this.rpmFilter.setQ(ShooterPID.kalmanQ);
        this.rpmFilter.setR(ShooterPID.kalmanR);

        double current = this.rpmFilter.update(this.motors.getVelocity(28.0));
        double currentLaunchSpeed = Regressions.rpmToVelocity(current);

        ProjectileMotion solution;
        if (ShooterPID.useConvergence)
            solution = ProjectileMotion.calculateConvergence(ProjectileMotion.getTarget(), currentLaunchSpeed);
        else
            solution = ProjectileMotion.calculate(ProjectileMotion.getTarget(), currentLaunchSpeed);

        this.targetVelocity = solution.launchVelocity;
        double targetRPM = Regressions.velocityToRpm(targetVelocity);

        double response = this.pid.calculate(current, targetRPM);
        double feedforward = ShooterPID.kS * Math.signum(targetRPM) + ShooterPID.kV * targetRPM;

        feedforward *= TaskOpMode.Runtime.voltageCompensation;
        if (targetRPM != 0) response += feedforward;

        if (this.state == State.Cruising) this.motors.setPower(response);
        else {
            if (currentLaunchSpeed < targetVelocity) this.motors.setPower(1);
            else this.motors.setPower(response);
        }

        double hoodAngle = Regressions.launchAngleToHoodAngle(solution.launchAngle);

        double servoPosition = this.calculateHoodAngleForDegrees(hoodAngle);
        if (servoPosition < 0) servoPosition = 0;
        if (servoPosition > 0.37) servoPosition = 0.37;
        hoodServo.setPosition(servoPosition);

        double quadraturePosition = quadrature.getPosition() / 4096 * 360;
        response = this.turretpid.calculate(quadraturePosition, solution.yawDirection);
        if (Math.abs(quadraturePosition - solution.yawDirection) > 1) {
            response += ZeroTurret.TurretPID.kS * Math.signum(solution.yawDirection - quadraturePosition);
        }
        this.leftTurretServo.setPower(response);
        this.rightTurretServo.setPower(response);

        if (useTelemetry) {
            telemetry.addData("launchAngle", solution.launchAngle);
            telemetry.addData("hoodAngle", hoodAngle);
            telemetry.addData("currentLaunchVelocity", currentLaunchSpeed);
            telemetry.addData("targetVelocity", targetVelocity);
            telemetry.addData("Flywheel RPM", current);
        }
    }

    public double calculateHoodAngleForDegrees(double degrees) {
        return ((90 - degrees) - ServoConstants.Ratios.hoodZeroAngle) / (ServoConstants.Ratios.hoodRatio * ServoConstants.Ratios.hoodAngle);
    }
}
