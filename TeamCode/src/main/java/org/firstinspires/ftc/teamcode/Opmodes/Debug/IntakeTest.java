package org.firstinspires.ftc.teamcode.Opmodes.Debug;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.lioncore.hardware.LionMotor;
import org.firstinspires.ftc.teamcode.lioncore.tasks.TaskOpMode;
import org.firstinspires.ftc.teamcode.parameters.MotorConstants;

@TeleOp
public class IntakeTest extends OpMode{

    private LionMotor intakeMotor;
    private LionMotor transferMotor;

    @Config
    public static class IntakeTransferTest {
        public static double intakePower = 0;
        public static double transferPower = 0;
        public static boolean useIntakeForBoth = false;
    }

    public void init(){
        this.intakeMotor = LionMotor.withoutEncoder(hardwareMap, MotorConstants.Names.intakeMotor);
        this.transferMotor = LionMotor.withoutEncoder(hardwareMap, MotorConstants.Names.transferMotor);
        this.intakeMotor.setReversed(MotorConstants.Reversed.intakeMotor);
        this.transferMotor.setReversed(MotorConstants.Reversed.transferMotor);
        this.intakeMotor.setZPB(MotorConstants.ZPB.intakeMotors);
        this.transferMotor.setZPB(MotorConstants.ZPB.intakeMotors);
    }

    public void loop() {
        this.intakeMotor.setPower(IntakeTransferTest.intakePower);

        if (IntakeTransferTest.useIntakeForBoth) this.transferMotor.setPower(IntakeTransferTest.intakePower);
        else this.transferMotor.setPower(IntakeTransferTest.transferPower);
    }

}