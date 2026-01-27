package org.firstinspires.ftc.teamcode.Opmodes.Debug;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.lioncore.hardware.LionMotor;
import org.firstinspires.ftc.teamcode.lioncore.hardware.LionServo;
import org.firstinspires.ftc.teamcode.parameters.MotorConstants;
import org.firstinspires.ftc.teamcode.parameters.ServoConstants;

@TeleOp
public class SwerveTuning extends LinearOpMode {

    @Config
    public static class SwerveConstants {

        public static double rfp = 0;
        public static double rfa = 0;
        public static double lfp = 0;
        public static double lfa = 0;
        public static double rrp = 0;
        public static double rra = 0;
        public static double lrp = 0;
        public static double lra = 0;

    }

    @Override
    public void runOpMode() {

        if (isStopRequested()) return;
        waitForStart();

        LionMotor rightFront = LionMotor.withoutEncoder(hardwareMap, MotorConstants.Names.rightFront);
        LionMotor leftFront = LionMotor.withoutEncoder(hardwareMap, MotorConstants.Names.leftFront);
        LionMotor rightRear = LionMotor.withoutEncoder(hardwareMap, MotorConstants.Names.rightRear);
        LionMotor leftRear = LionMotor.withoutEncoder(hardwareMap, MotorConstants.Names.leftRear);
        rightFront.setReversed(MotorConstants.Reversed.rf);
        rightFront.setZPB(MotorConstants.ZPB.driveMotors);
        leftFront.setReversed(MotorConstants.Reversed.lf);
        leftFront.setZPB(MotorConstants.ZPB.driveMotors);
        rightRear.setReversed(MotorConstants.Reversed.rr);
        rightRear.setZPB(MotorConstants.ZPB.driveMotors);
        leftRear.setReversed(MotorConstants.Reversed.lr);
        leftRear.setZPB(MotorConstants.ZPB.driveMotors);
        LionServo rightFrontServo = LionServo.single(hardwareMap, ServoConstants.Names.rightFront, 0.5);
        LionServo leftFrontServo = LionServo.single(hardwareMap, ServoConstants.Names.leftFront, 0.5);
        LionServo rightRearServo = LionServo.single(hardwareMap, ServoConstants.Names.rightRear, 0.5);
        LionServo leftRearServo = LionServo.single(hardwareMap, ServoConstants.Names.leftRear, 0.5);

        while (opModeIsActive()) {
            rightFront.setPower(SwerveConstants.rfp);
            leftFront.setPower(SwerveConstants.lfp);
            rightRear.setPower(SwerveConstants.rrp);
            leftRear.setPower(SwerveConstants.lrp);

            rightFrontServo.setPosition(SwerveConstants.rfa);
            leftFrontServo.setPosition(SwerveConstants.lfa);
            rightRearServo.setPosition(SwerveConstants.rra);
            leftRearServo.setPosition(SwerveConstants.lra);
        }

    }
}
