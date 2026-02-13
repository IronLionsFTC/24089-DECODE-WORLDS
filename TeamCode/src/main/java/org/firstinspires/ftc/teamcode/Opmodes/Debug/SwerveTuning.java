package org.firstinspires.ftc.teamcode.Opmodes.Debug;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.lioncore.hardware.AbsoluteEncoder;
import org.firstinspires.ftc.teamcode.lioncore.hardware.LionCRServo;
import org.firstinspires.ftc.teamcode.lioncore.hardware.LionMotor;
import org.firstinspires.ftc.teamcode.parameters.MotorConstants;
import org.firstinspires.ftc.teamcode.parameters.ServoConstants;
import org.firstinspires.ftc.teamcode.parameters.Zeroing;

@TeleOp
public class SwerveTuning extends LinearOpMode {

    @Config
    public static class SwerveConstants {

        public static double rfp = 0;
        public static double lfp = 0;
        public static double rrp = 0;
        public static double lrp = 0;

        public static double rfmp = 0;
        public static double lfmp = 0;
        public static double rrmp = 0;
        public static double lrmp = 0;

    }

    @Override
    public void runOpMode() {

        if (isStopRequested()) return;
        waitForStart();

        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        LionCRServo rightFrontServo = new LionCRServo(hardwareMap, ServoConstants.Names.rightFront);
        LionCRServo leftFrontServo = new LionCRServo(hardwareMap, ServoConstants.Names.leftFront);
        LionCRServo rightRearServo = new LionCRServo(hardwareMap, ServoConstants.Names.rightRear);
        LionCRServo leftRearServo = new LionCRServo(hardwareMap, ServoConstants.Names.leftRear);

        AbsoluteEncoder rightFrontAnalog = new AbsoluteEncoder(hardwareMap, Zeroing.Names.rightFrontAnalog, Zeroing.ZeroPositions.rightFront);
        AbsoluteEncoder leftFrontAnalog = new AbsoluteEncoder(hardwareMap, Zeroing.Names.leftFrontAnalog, Zeroing.ZeroPositions.leftFront);
        AbsoluteEncoder rightRearAnalog = new AbsoluteEncoder(hardwareMap, Zeroing.Names.rightRearAnalog, Zeroing.ZeroPositions.rightRear);
        AbsoluteEncoder leftRearAnalog = new AbsoluteEncoder(hardwareMap, Zeroing.Names.leftRearAnalog, Zeroing.ZeroPositions.leftRear);

        LionMotor rightFront = LionMotor.withEncoder(hardwareMap, MotorConstants.Names.rightFront);
        LionMotor leftFront = LionMotor.withEncoder(hardwareMap, MotorConstants.Names.leftFront);
        LionMotor rightRear = LionMotor.withEncoder(hardwareMap, MotorConstants.Names.rightRear);
        LionMotor leftRear = LionMotor.withEncoder(hardwareMap, MotorConstants.Names.leftRear);

        rightFront.setReversed(MotorConstants.Reversed.rf);
        rightFront.setZPB(MotorConstants.ZPB.driveMotors);
        leftFront.setReversed(MotorConstants.Reversed.lf);
        leftFront.setZPB(MotorConstants.ZPB.driveMotors);
        rightRear.setReversed(MotorConstants.Reversed.rr);
        rightRear.setZPB(MotorConstants.ZPB.driveMotors);
        leftRear.setReversed(MotorConstants.Reversed.lr);
        leftRear.setZPB(MotorConstants.ZPB.driveMotors);
        rightFront.setReverseEncoder(true);
        rightRear.setReverseEncoder(true);

        rightFrontAnalog.read();
        leftFrontAnalog.read();
        rightRearAnalog.read();
        leftRearAnalog.read();

        rightFront.resetPositionTo(Zeroing.podAngle(rightFrontAnalog.position(), Zeroing.ZeroPositions.rightFront) * (4096.0 / 360.0));
        leftFront.resetPositionTo(Zeroing.podAngle(leftFrontAnalog.position(), Zeroing.ZeroPositions.leftFront) * (4096.0 / 360.0));
        rightRear.resetPositionTo(Zeroing.podAngle(rightRearAnalog.position(), Zeroing.ZeroPositions.rightRear) * (4096.0 / 360.0));
        leftRear.resetPositionTo(Zeroing.podAngle(leftRearAnalog.position(), Zeroing.ZeroPositions.leftRear) * (4096.0 / 360.0));

        while (opModeIsActive()) {
            rightFrontServo.setPower(SwerveConstants.rfp);
            leftFrontServo.setPower(SwerveConstants.lfp);
            rightRearServo.setPower(SwerveConstants.rrp);
            leftRearServo.setPower(SwerveConstants.lrp);

            rightFrontAnalog.read();
            leftFrontAnalog.read();
            rightRearAnalog.read();
            leftRearAnalog.read();

            rightFront.setPower(SwerveConstants.rfmp);
            leftFront.setPower(SwerveConstants.lfmp);
            rightRear.setPower(SwerveConstants.rrmp);
            leftRear.setPower(SwerveConstants.lrmp);

            telemetry.addData("rightFront", Zeroing.podAngle(rightFrontAnalog.position(), Zeroing.ZeroPositions.rightFront));
            telemetry.addData("leftFront", Zeroing.podAngle(leftFrontAnalog.position(), Zeroing.ZeroPositions.leftFront));
            telemetry.addData("rightRear", Zeroing.podAngle(rightRearAnalog.position(), Zeroing.ZeroPositions.rightRear));
            telemetry.addData("leftRear", Zeroing.podAngle(leftRearAnalog.position(), Zeroing.ZeroPositions.leftRear));

            telemetry.addData("rightFrontAbs", rightFrontAnalog.position());
            telemetry.addData("leftFrontAbs", leftFrontAnalog.position());
            telemetry.addData("rightRearAbs", rightRearAnalog.position());
            telemetry.addData("leftRearAbs", leftRearAnalog.position());

            telemetry.addData("rightFrontQuadrature", Zeroing.polarQuadrature(rightFront.getPosition()));
            telemetry.addData("leftFrontQuadrature", Zeroing.polarQuadrature(leftFront.getPosition()));
            telemetry.addData("rightRearQuadrature", Zeroing.polarQuadrature(rightRear.getPosition()));
            telemetry.addData("leftRearQuadrature", Zeroing.polarQuadrature(leftRear.getPosition()));
            telemetry.update();
        }

    }
}
