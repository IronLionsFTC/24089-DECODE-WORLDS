package org.firstinspires.ftc.teamcode.Opmodes.Debug;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.lioncore.hardware.LionServo;
import org.firstinspires.ftc.teamcode.parameters.ServoConstants;

@TeleOp
public class ServoModeTuning extends LinearOpMode {
    @Config
    public static class Positions {
        public static double rightFront = 0;
        public static double leftFront = 0;
        public static double rightRear = 0;
        public static double leftRear = 0;

        public static double rfz = -3;
        public static double lfz = 5;
        public static double rrz = -3;
        public static double lrz = 0;
    }

    private double toPosition(double degrees, double podOffset) {
        double servoRange = (double) (270 * 23) / 16 * 180 / 150;
        double position = ((degrees + podOffset) / servoRange) + 0.5;
        while (position > 1) position -= 1;
        while (position < 0) position += 1;
        return position;
    }

    public void runOpMode() {

        LionServo rightFront = LionServo.single(hardwareMap, ServoConstants.Names.rightFront, 0.5);
        LionServo leftFront = LionServo.single(hardwareMap, ServoConstants.Names.leftFront, 0.5);
        LionServo rightRear = LionServo.single(hardwareMap, ServoConstants.Names.rightRear, 0.5);
        LionServo leftRear = LionServo.single(hardwareMap, ServoConstants.Names.leftRear, 0.5);

        if (isStopRequested()) return;
        waitForStart();

        while (opModeIsActive()) {
            rightFront.setPosition(toPosition(Positions.rightFront, Positions.rfz));
            leftFront.setPosition(toPosition(Positions.leftFront, Positions.lfz));
            rightRear.setPosition(toPosition(Positions.rightRear, Positions.rrz));
            leftRear.setPosition(toPosition(Positions.leftRear, Positions.lrz));
        }
    }
}
