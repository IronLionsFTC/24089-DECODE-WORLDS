package org.firstinspires.ftc.teamcode.Opmodes.Debug;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.lioncore.hardware.AbsoluteEncoder;
import org.firstinspires.ftc.teamcode.lioncore.system.ConstantsStorage;
import org.firstinspires.ftc.teamcode.parameters.Zeroing;

public class ZeroSwerve extends LinearOpMode {

    public void runOpMode() {

        if (isStopRequested()) return;

        AbsoluteEncoder rightFrontAnalog = new AbsoluteEncoder(hardwareMap, Zeroing.Names.rightFrontAnalog);
        AbsoluteEncoder leftFrontAnalog = new AbsoluteEncoder(hardwareMap, Zeroing.Names.leftFrontAnalog);
        AbsoluteEncoder rightRearAnalog = new AbsoluteEncoder(hardwareMap, Zeroing.Names.rightRearAnalog);
        AbsoluteEncoder leftRearAnalog = new AbsoluteEncoder(hardwareMap, Zeroing.Names.leftRearAnalog);

        waitForStart();

        rightFrontAnalog.read();
        leftFrontAnalog.read();
        rightRearAnalog.read();
        leftRearAnalog.read();

        ConstantsStorage.save("rf", rightFrontAnalog.position());
        ConstantsStorage.save("lf", leftFrontAnalog.position());
        ConstantsStorage.save("rr", rightRearAnalog.position());
        ConstantsStorage.save("lr", leftRearAnalog.position());

        while (opModeIsActive()) {

        }
    }
}
