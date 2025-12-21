package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.DashboardCore;
import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp
public class HoodTuning extends OpMode {

    DcMotorEx leftShooterMotor;
    DcMotorEx rightShooterMotor;
    Servo hoodServo;

    @Override
    public void init() {
        this.hoodServo = hardwareMap.get(Servo.class, "hoodServo");
        this.leftShooterMotor = hardwareMap.get(DcMotorEx.class, "leftShooterMotor");
        this.rightShooterMotor = hardwareMap.get(DcMotorEx.class, "rightShooterMotor");
        this.leftShooterMotor.setDirection(DcMotorSimple.Direction.REVERSE);
    }

    @Config
    public static class MotorPositions {
        public static double ShooterPower = 0;
        public static double hoodServoPosition = 0;

    }

    @Override
    public void loop() {
        this.hoodServo.setPosition(MotorPositions.hoodServoPosition);
        this.leftShooterMotor.setPower(MotorPositions.ShooterPower);
        this.rightShooterMotor.setPower(MotorPositions.ShooterPower);
        telemetry.addData("Shooter Velocity", leftShooterMotor.getVelocity());
        telemetry.update();

    }

}
