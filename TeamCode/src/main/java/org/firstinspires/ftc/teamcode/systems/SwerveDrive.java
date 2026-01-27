package org.firstinspires.ftc.teamcode.systems;

import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.lioncore.hardware.LionMotor;
import org.firstinspires.ftc.teamcode.lioncore.hardware.LionServo;
import org.firstinspires.ftc.teamcode.lioncore.math.types.Position;
import org.firstinspires.ftc.teamcode.lioncore.math.types.Vector;
import org.firstinspires.ftc.teamcode.lioncore.systems.SystemBase;
import org.firstinspires.ftc.teamcode.parameters.MotorConstants;
import org.firstinspires.ftc.teamcode.parameters.ServoConstants;

import java.util.function.DoubleSupplier;

public class SwerveDrive extends SystemBase {

    private double servoRatio = 0.65625;

    private GoBildaPinpointDriver pinpoint;
    public LionMotor rightFront;
    public LionServo rightFrontServo;
    public LionMotor leftFront;
    public LionServo leftFrontServo;
    public LionMotor rightRear;
    public LionServo rightRearServo;
    public LionMotor leftRear;
    public LionServo leftRearServo;

    private Position startPosition;

    private DoubleSupplier joystickX;
    private DoubleSupplier joystickY;
    private DoubleSupplier joystickH;

    public SwerveDrive(Position startPosition) {
        this.startPosition = startPosition;
        this.joystickX = () -> 0;
        this.joystickY = () -> 0;
        this.joystickH = () -> 0;
    }

    public SwerveDrive(Position startPosition, DoubleSupplier x, DoubleSupplier y, DoubleSupplier h) {
        this.startPosition = startPosition;
        this.joystickX = x;
        this.joystickY = y;
        this.joystickH = h;
    }

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
        this.rightFrontServo = LionServo.single(hardwareMap, ServoConstants.Names.rightFront, 0.5);
        this.leftFrontServo = LionServo.single(hardwareMap, ServoConstants.Names.leftFront, 0.5);
        this.rightRearServo = LionServo.single(hardwareMap, ServoConstants.Names.rightRear, 0.5);
        this.leftRearServo = LionServo.single(hardwareMap, ServoConstants.Names.leftRear, 0.5);
    }

    @Override
    public void init() {
        this.pinpoint.resetPosAndIMU();
        this.pinpoint.setPosition(startPosition.pose());
        this.pinpoint.setHeading(startPosition.heading, AngleUnit.DEGREES);
    }

    @Override
    public void update(Telemetry telemetry) {

        Vector input = Vector.cartesian(joystickX.getAsDouble(), joystickY.getAsDouble());

    }

    /**
     * Optimise the movement of a swerve pod to the position it is suppose to be at
     * @param polar
     * @return
     */
    private boolean applyDirectionToServo(LionServo servo, double polar) {

        double current = servo.getPosition() * (21.0 / 16.0);
        double currentDegrees = (current - 0.5) * servoRatio * 360;
        double reversed = polar + 180;
        while (currentDegrees > 180) currentDegrees -= 360;
        while (currentDegrees < -180) currentDegrees += 360;
        while (reversed > 180) reversed -= 360;
        while (reversed < -180) reversed += 360;

        double forwardError = Math.abs(polar - current);
        double reversedError = Math.abs(reversed - current);

        if (forwardError >= reversedError) {
            servo.setPosition(reversed / (360 * servoRatio) + 0.5);
            return true;
        } else {
            servo.setPosition(polar / (360 * servoRatio) + 0.5);
            return false;
        }
    }
}