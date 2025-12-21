package org.firstinspires.ftc.teamcode.systems;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.lioncore.hardware.LionMotor;
import org.firstinspires.ftc.teamcode.lioncore.hardware.LionServo;
import org.firstinspires.ftc.teamcode.lioncore.math.pid.PID;
import org.firstinspires.ftc.teamcode.lioncore.systems.SystemBase;
import org.firstinspires.ftc.teamcode.parameters.MotorConstants;
import org.firstinspires.ftc.teamcode.parameters.ServoConstants;

public class Shooter extends SystemBase {

    // Hardware
    private LionMotor motors;
    private LionServo hoodServo;

    // Control
    private PID pid;

    // PID constants
    @Config
    public static class ShooterPID {
        public static double P = 0;
        public static double I = 0;
        public static double D = 0;
        public static double F = 0;
        public static double targetOverride = 0;
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
                ShooterPID.P,
                ShooterPID.P
        );
        this.motors.setReversed(MotorConstants.Reversed.leftShooterMotor, MotorConstants.Reversed.rightShooterMotor);
        this.motors.setZPB(MotorConstants.ZPB.shooterMotors);
    }

    @Override
    public void update(Telemetry telemetry) {
        this.pid.setConstants(
                ShooterPID.P,
                ShooterPID.P,
                ShooterPID.P
        );
    }
}
