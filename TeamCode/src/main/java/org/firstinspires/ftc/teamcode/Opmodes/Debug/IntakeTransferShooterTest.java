package org.firstinspires.ftc.teamcode.Opmodes.Debug;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.lioncore.hardware.LionMotor;
import org.firstinspires.ftc.teamcode.lioncore.tasks.TaskOpMode;
import org.firstinspires.ftc.teamcode.parameters.MotorConstants;

@TeleOp
public class IntakeTransferShooterTest extends OpMode {

    private LionMotor intakeMotor;
    private LionMotor transferMotor;
    private LionMotor outtakeMotor1;
    private LionMotor outtakeMotor2;


    @Config
    public static class AllMotors {
        public static double intakePower = 0;
        public static double outtakePower = 0;
    }

    public void init() {
        this.intakeMotor = LionMotor.withoutEncoder(hardwareMap, MotorConstants.Names.intakeMotor);
        this.transferMotor = LionMotor.withoutEncoder(hardwareMap, MotorConstants.Names.transferMotor);
        this.outtakeMotor1 = LionMotor.withoutEncoder(hardwareMap, MotorConstants.Names.rightShooterMotor);
        this.outtakeMotor2 = LionMotor.withEncoder(hardwareMap, MotorConstants.Names.leftShooterMotor);
        this.intakeMotor.setReversed(MotorConstants.Reversed.intakeMotor);
        this.transferMotor.setReversed(MotorConstants.Reversed.transferMotor);
        this.intakeMotor.setZPB(MotorConstants.ZPB.intakeMotors);
        this.transferMotor.setZPB(MotorConstants.ZPB.intakeMotors);
        this.outtakeMotor1.setReversed(MotorConstants.Reversed.rightShooterMotor);


    }

    public void loop() {
        this.intakeMotor.setPower(IntakeTest.IntakeTransferTest.intakePower);
        this.outtakeMotor2.setPower(-1*IntakeTransferShooterTest.AllMotors.outtakePower);
        this.outtakeMotor1.setPower(-1*IntakeTransferShooterTest.AllMotors.outtakePower);
        this.transferMotor.setPower(IntakeTest.IntakeTransferTest.intakePower);

        if (IntakeTest.IntakeTransferTest.useIntakeForBoth) this.transferMotor.setPower(IntakeTest.IntakeTransferTest.intakePower);
        else this.transferMotor.setPower(IntakeTest.IntakeTransferTest.transferPower);
    }
}

