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
import org.firstinspires.ftc.teamcode.lioncore.math.types.Vector3;
import org.firstinspires.ftc.teamcode.lioncore.system.ConstantsStorage;
import org.firstinspires.ftc.teamcode.lioncore.systems.SystemBase;
import org.firstinspires.ftc.teamcode.lioncore.tasks.TaskOpMode;
import org.firstinspires.ftc.teamcode.parameters.MotorConstants;
import org.firstinspires.ftc.teamcode.parameters.ServoConstants;
import org.firstinspires.ftc.teamcode.projectileMotion.ProjectileMotion;
import org.firstinspires.ftc.teamcode.projectileMotion.Regressions;

import java.util.ArrayList;

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
    public double currentLaunchSpeed;
    public double targetHood;
    public ArrayList<Double> rpmBuffer;

    public Shooter(Encoder intakeEncoderForTurret) {
        this.quadrature = intakeEncoderForTurret;
        this.targetVelocity = 0;
        this.targetHood = 0;
        this.target = new Vector3(0, 0, 3500);
        this.rpmBuffer = new ArrayList<>();
    }

    // PID constants
    @Config
    public static class ShooterPID {
        public static double P = 0.003;
        public static double I = 0;
        public static double D = 0.00003;
        public static double kS = 0.0;
        public static double kV = 0.0001;

        public static double targetXFar = 0;
        public static double targetYFar = 150;
        public static double targetZFar = 1100;
        public static double targetXClose = 0;
        public static double targetYClose = -50;
        public static double targetZClose = 1275;

        public static boolean useConvergence = true;

        public static double overPower = 1;
        public static double intakePower = 1;

        public static double expectedDrop = 0.6;

        public static double hoodAngle = 0;
        public static double launchVelocity = 0;

        public static double velocityScale = 2000;
        public static boolean useVComp = true;
        public static boolean useMinimum = true;
    }

    @Override
    public void loadHardware(HardwareMap hardwareMap) {

        ServoConstants.Zero.turret = ConstantsStorage.get("turretZeroVoltage", ServoConstants.Zero.turret);
        this.motors = LionMotor.masterSlaves(hardwareMap, MotorConstants.Names.leftShooterMotor, MotorConstants.Names.rightShooterMotor);
        this.hoodServo = LionServo.single(hardwareMap, ServoConstants.Names.hoodServo, ServoConstants.Zero.hood);

        AbsoluteEncoder absolute = new AbsoluteEncoder(hardwareMap, "turretAbsolute");
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
        this.rightTurretServo.setReversed(true);
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

        double sum = this.motors.getVelocity(28.0);
        if (Double.isNaN(sum)) sum = 0;
        currentLaunchSpeed = Regressions.rpmToVelocity(sum);

        ProjectileMotion solution;
        if (ShooterPID.useConvergence)
            solution = ProjectileMotion.calculateConvergence(ProjectileMotion.getTarget(), currentLaunchSpeed / ShooterPID.overPower);
        else
            solution = ProjectileMotion.calculate(ProjectileMotion.getTarget(), currentLaunchSpeed / ShooterPID.overPower);

        this.targetVelocity = solution.velocity * ShooterPID.overPower;
        if (ShooterPID.launchVelocity != 0) targetVelocity = ShooterPID.launchVelocity;

        if (this.targetVelocity < 4000 || Double.isNaN(targetVelocity)) targetVelocity = 8500;
        double targetRPM = Regressions.velocityToRpm(targetVelocity);

        double response = this.pid.calculate(sum, targetRPM);
        double feedforward = ShooterPID.kS * Math.signum(targetRPM) + ShooterPID.kV * targetRPM;

        if (!Double.isNaN(TaskOpMode.Runtime.voltageCompensation)) feedforward *= TaskOpMode.Runtime.voltageCompensation;
        if (targetRPM != 0) response += feedforward;

        telemetry.addData("RESPONSE", response);

        if (response < 0) response = 0;

        if (this.state == State.Cruising) this.motors.setPower(response);
        else {
            this.motors.setPower(response);
        }

        double hoodAngle = Regressions.launchAngleToHoodAngle(solution.altitude);
        if (ShooterPID.hoodAngle != 0) hoodAngle = Regressions.launchAngleToHoodAngle(ShooterPID.hoodAngle);

        double servoPosition = this.calculateHoodAngleForDegrees(hoodAngle);
        if (servoPosition > 1) servoPosition = 1;
        if (servoPosition < 1 - 0.45) servoPosition = 1 - 0.45;
        hoodServo.setPosition(servoPosition);

        double quadraturePosition = quadrature.getPosition() / 4096 * 360;
        response = this.turretpid.calculate(quadraturePosition, solution.azimuth);
        if (Math.abs(quadraturePosition - solution.azimuth) > 1) {
            response += ZeroTurret.TurretPID.kS * Math.signum(solution.azimuth - quadraturePosition);
        }
        this.leftTurretServo.setPower(response);
        this.rightTurretServo.setPower(response);

        telemetry.addData("currentLaunchVelocity", currentLaunchSpeed);

        if (useTelemetry) {
            telemetry.addData("launchAngle", solution.altitude);
            telemetry.addData("hoodAngle", hoodAngle);
            telemetry.addData("targetVelocity", targetVelocity);
            telemetry.addData("Flywheel RPM", sum);
        }
    }

    public double calculateHoodAngleForDegrees(double degrees) {
        return 1 - (((90 - degrees) - (90 - ServoConstants.Ratios.hoodZeroAngle)) / (ServoConstants.Ratios.hoodRatio * ServoConstants.Ratios.hoodAngle) + 0.015);
    }

    public boolean atSpeed() {
        return Math.abs(currentLaunchSpeed - targetVelocity) < 300 && currentLaunchSpeed > 3000;
    }
}
