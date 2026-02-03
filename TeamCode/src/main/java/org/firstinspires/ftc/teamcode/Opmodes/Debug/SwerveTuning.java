package org.firstinspires.ftc.teamcode.Opmodes.Debug;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.lioncore.hardware.AbsoluteEncoder;
import org.firstinspires.ftc.teamcode.lioncore.hardware.LionCRServo;
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

    }

    @Override
    public void runOpMode() {

        if (isStopRequested()) return;
        waitForStart();
        LionCRServo rightFrontServo = new LionCRServo(hardwareMap, ServoConstants.Names.rightFront);
        LionCRServo leftFrontServo = new LionCRServo(hardwareMap, ServoConstants.Names.leftFront);
        LionCRServo rightRearServo = new LionCRServo(hardwareMap, ServoConstants.Names.rightRear);
        LionCRServo leftRearServo = new LionCRServo(hardwareMap, ServoConstants.Names.leftRear);

        AbsoluteEncoder rightFrontAnalog = new AbsoluteEncoder(hardwareMap, Zeroing.Names.rightFrontAnalog, Zeroing.ZeroPositions.rightFront);
        AbsoluteEncoder leftFrontAnalog = new AbsoluteEncoder(hardwareMap, Zeroing.Names.leftFrontAnalog, Zeroing.ZeroPositions.leftFront);
        AbsoluteEncoder rightRearAnalog = new AbsoluteEncoder(hardwareMap, Zeroing.Names.rightRearAnalog, Zeroing.ZeroPositions.rightRear);
        AbsoluteEncoder leftRearAnalog = new AbsoluteEncoder(hardwareMap, Zeroing.Names.leftRearAnalog, Zeroing.ZeroPositions.leftRear);

        while (opModeIsActive()) {
            rightFrontServo.setPower(SwerveConstants.rfp);
            leftFrontServo.setPower(SwerveConstants.lfp);
            rightRearServo.setPower(SwerveConstants.rrp);
            leftRearServo.setPower(SwerveConstants.lrp);

            rightFrontAnalog.read();
            leftFrontAnalog.read();
            rightRearAnalog.read();
            leftRearAnalog.read();

            telemetry.addData("rightFront", rightFrontAnalog.position());
            telemetry.addData("leftFront", leftFrontAnalog.position());
            telemetry.addData("rightRear", rightRearAnalog.position());
            telemetry.addData("leftRear", leftRearAnalog.position());
        }

    }
}
