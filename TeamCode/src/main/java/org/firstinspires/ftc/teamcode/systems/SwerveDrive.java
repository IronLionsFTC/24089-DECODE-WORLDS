package org.firstinspires.ftc.teamcode.systems;

import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.lioncore.hardware.LionMotor;
import org.firstinspires.ftc.teamcode.lioncore.hardware.LionServo;
import org.firstinspires.ftc.teamcode.lioncore.systems.SystemBase;
import org.firstinspires.ftc.teamcode.parameters.MotorConstants;
import org.firstinspires.ftc.teamcode.parameters.ServoConstants;

public class SwerveDrive extends SystemBase {

    private GoBildaPinpointDriver pinpoint;

    public LionMotor rightFront;
    public LionServo rightFrontServo;
    public LionMotor leftFront;
    public LionServo leftFrontServo;
    public LionMotor rightRear;
    public LionServo rightRearServo;
    public LionMotor leftRear;
    public LionServo leftRearServo;

    @Override
    public void loadHardware(HardwareMap hardwareMap) {
        this.pinpoint = hardwareMap.get(GoBildaPinpointDriver.class, "pinpoint");
        this.rightFront = LionMotor.withoutEncoder(hardwareMap, MotorConstants.Names.rightFront);
        this.leftFront = LionMotor.withoutEncoder(hardwareMap, MotorConstants.Names.leftFront);
        this.rightRear = LionMotor.withoutEncoder(hardwareMap, MotorConstants.Names.rightRear);
        this.leftRear = LionMotor.withoutEncoder(hardwareMap, MotorConstants.Names.leftRear);
        this.rightFront.setReversed(MotorConstants.Reversed.rf);
        this.rightFront.setZPB(MotorConstants.ZPB.driveMotors);
        this.leftFront.setReversed(MotorConstants.Reversed.lf);
        this.leftFront.setZPB(MotorConstants.ZPB.driveMotors);
        this.rightRear.setReversed(MotorConstants.Reversed.rr);
        this.rightRear.setZPB(MotorConstants.ZPB.driveMotors);
        this.leftRear.setReversed(MotorConstants.Reversed.lr);
        this.leftRear.setZPB(MotorConstants.ZPB.driveMotors);
        this.rightFrontServo = LionServo.single(hardwareMap, ServoConstants.Names.rightFront, 0);
        this.leftFrontServo = LionServo.single(hardwareMap, ServoConstants.Names.leftFront, 0);
        this.rightRearServo = LionServo.single(hardwareMap, ServoConstants.Names.rightRear, 0);
        this.leftRearServo = LionServo.single(hardwareMap, ServoConstants.Names.leftRear, 0);
    }
}
