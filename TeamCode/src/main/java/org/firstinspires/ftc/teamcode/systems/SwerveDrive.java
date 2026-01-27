package org.firstinspires.ftc.teamcode.systems;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.UnnormalizedAngleUnit;
import org.firstinspires.ftc.teamcode.lioncore.hardware.LionMotor;
import org.firstinspires.ftc.teamcode.lioncore.hardware.LionServo;
import org.firstinspires.ftc.teamcode.lioncore.math.pid.PID;
import org.firstinspires.ftc.teamcode.lioncore.math.types.Position;
import org.firstinspires.ftc.teamcode.lioncore.math.types.Vector;
import org.firstinspires.ftc.teamcode.lioncore.systems.SystemBase;
import org.firstinspires.ftc.teamcode.parameters.MotorConstants;
import org.firstinspires.ftc.teamcode.parameters.ServoConstants;

import java.util.function.DoubleSupplier;

public class SwerveDrive extends SystemBase {

    private GoBildaPinpointDriver pinpoint;
    private SwervePod rightFront;
    private SwervePod leftFront;
    private SwervePod rightRear;
    private SwervePod leftRear;

    private Position startPosition;

    private DoubleSupplier joystickX;
    private DoubleSupplier joystickY;
    private DoubleSupplier joystickH;

    private double targetHeading;
    private PID headingController;
    private boolean turning;

    @Config
    public static class SwervePID {
        public static double P = 0;
        public static double I = 0;
        public static double D = 0;
    }

    public SwerveDrive(Position startPosition) {
        this.startPosition = startPosition;
        this.joystickX = () -> 0;
        this.joystickY = () -> 0;
        this.joystickH = () -> 0;
        this.headingController = new PID(0, 0, 0);
    }

    public SwerveDrive(Position startPosition, DoubleSupplier x, DoubleSupplier y, DoubleSupplier h) {
        this.startPosition = startPosition;
        this.joystickX = x;
        this.joystickY = y;
        this.joystickH = h;
        this.headingController = new PID(0, 0, 0);
    }

    @Override
    public void loadHardware(HardwareMap hardwareMap) {
        this.pinpoint = hardwareMap.get(GoBildaPinpointDriver.class, "pinpoint");
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
        this.rightFront = new SwervePod(rightFront, LionServo.single(hardwareMap, ServoConstants.Names.rightFront, 0.5), Vector.cartesian(1, 1));
        this.leftFront = new SwervePod(leftFront, LionServo.single(hardwareMap, ServoConstants.Names.leftFront, 0.5), Vector.cartesian(-1, 1));
        this.rightRear = new SwervePod(rightRear, LionServo.single(hardwareMap, ServoConstants.Names.rightRear, 0.5), Vector.cartesian(1, -1));
        this.leftRear = new SwervePod(leftRear, LionServo.single(hardwareMap, ServoConstants.Names.leftRear, 0.5), Vector.cartesian(-1, -1));
    }

    @Override
    public void init() {
        this.pinpoint.resetPosAndIMU();
        this.pinpoint.setPosition(startPosition.pose());
        this.pinpoint.setHeading(startPosition.heading, AngleUnit.DEGREES);
        this.targetHeading = startPosition.heading;
        this.turning = false;
    }

    @Override
    public void update(Telemetry telemetry) {

        this.headingController.setConstants(
            SwervePID.P,
            SwervePID.I,
            SwervePID.D
        );

        this.pinpoint.update();
        double current = this.pinpoint.getHeading(AngleUnit.DEGREES);
        double error = angleDifference(this.targetHeading, current);
        double response = this.headingController.calculate(error, 0);

        double x = joystickX.getAsDouble();
        double y = joystickY.getAsDouble();
        double h = joystickH.getAsDouble();

        if (h == 0 && !turning) {
            h = response;
        } else {
            this.turning = true;
            this.targetHeading = current;

            if (Math.abs(pinpoint.getHeadingVelocity(UnnormalizedAngleUnit.DEGREES)) < 5 && h == 0) {
                this.turning = false;
            }
        }

        Vector input = Vector.cartesian(x, y);

        this.rightFront.update(input, h);
        this.leftFront.update(input, h);
        this.rightRear.update(input, h);
        this.leftRear.update(input, h);

        telemetry.addData("Angle", input.polarDirection());

    }

    /**
     * Computes shortest angular difference from current to target in degrees.
     * Result is in range [-180, 180].
     */
    public static double angleDifference(double target, double current) {
        double delta = target - current;
        while (delta > 180) delta -= 360;
        while (delta < -180) delta += 360;
        return delta;
    }
}