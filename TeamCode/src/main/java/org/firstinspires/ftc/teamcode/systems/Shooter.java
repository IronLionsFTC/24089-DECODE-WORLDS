package org.firstinspires.ftc.teamcode.systems;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.lioncore.hardware.LionMotor;
import org.firstinspires.ftc.teamcode.lioncore.hardware.LionServo;
import org.firstinspires.ftc.teamcode.lioncore.math.pid.PID;
import org.firstinspires.ftc.teamcode.lioncore.systems.SystemBase;
import org.firstinspires.ftc.teamcode.lioncore.tasks.TaskOpMode;
import org.firstinspires.ftc.teamcode.parameters.MotorConstants;
import org.firstinspires.ftc.teamcode.parameters.ServoConstants;

public class Shooter extends SystemBase {

    // Hardware
    private LionMotor motors;
    private LionServo hoodServo;

    // Control
    private PID pid;
    public double targetRPM = 0;
    public double targetHood = 0 ;

    public Shooter() {
        this.targetRPM = 0;
        this.targetHood = 0;
    }

    // PID constants
    @Config
    public static class ShooterPID {
        public static double P = 0.003;
        public static double I = 0;
        public static double D = 0.00003;
        public static double kS = 0.07;
        public static double kV = 0.00017;
        public static double targetOverride = 0;
        public static double targetHood = 23;
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
        double target = (ShooterPID.targetOverride == 0) ? targetRPM : ShooterPID.targetOverride;
        double response = this.pid.calculate(current, target);

        double feedforward = ShooterPID.kS * Math.signum(target) + ShooterPID.kV * target;
        feedforward *= TaskOpMode.Runtime.voltageCompensation;

        if (target != 0) response += feedforward;

        this.motors.setPower(response);
        this.hoodServo.setPosition(calculateHoodAngleForDegrees(ShooterPID.targetHood));
    }

    public double getHoodAngleDegrees() {
        return this.hoodServo.getPosition() * ServoConstants.Ratios.hoodRatio * ServoConstants.Ratios.hoodAngle + ServoConstants.Ratios.hoodZeroAngle;
    }

    public double calculateHoodAngleForDegrees(double degrees) {
        return (degrees - ServoConstants.Ratios.hoodZeroAngle) / (ServoConstants.Ratios.hoodRatio * ServoConstants.Ratios.hoodAngle);
    }

    /**
     * Convert instantaneous RPM into launch velocity
     * @return velocity (mm/s)
     */
    public double rpmToVelocity(double rpm) {
        return (rpm * 0.00125699 + 2.15515) * 1000;
    }
}
