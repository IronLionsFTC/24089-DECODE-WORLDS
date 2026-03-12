package org.firstinspires.ftc.teamcode.systems;

import static android.os.SystemClock.sleep;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.robotcore.external.navigation.UnnormalizedAngleUnit;
import org.firstinspires.ftc.teamcode.lioncore.hardware.AbsoluteEncoder;
import org.firstinspires.ftc.teamcode.lioncore.hardware.LionCRServo;
import org.firstinspires.ftc.teamcode.lioncore.hardware.LionMotor;
import org.firstinspires.ftc.teamcode.lioncore.math.pid.PID;
import org.firstinspires.ftc.teamcode.lioncore.math.types.Position;
import org.firstinspires.ftc.teamcode.lioncore.math.types.Vector2;
import org.firstinspires.ftc.teamcode.lioncore.system.ConstantsStorage;
import org.firstinspires.ftc.teamcode.lioncore.systems.SystemBase;
import org.firstinspires.ftc.teamcode.parameters.MotorConstants;
import org.firstinspires.ftc.teamcode.parameters.ServoConstants;
import org.firstinspires.ftc.teamcode.parameters.Zeroing;

import java.util.function.DoubleSupplier;

public class SwerveDrive extends SystemBase {

    private GoBildaPinpointDriver pinpoint;
    private SwervePod rightFront;
    private SwervePod leftFront;
    private SwervePod rightRear;
    private SwervePod leftRear;

    private Position startPosition;

    private Vector2 driveVector;
    private DoubleSupplier heading;

    private double targetHeading;
    private PID headingController;
    private boolean turning;

    private final boolean xPattern;

    public static class PinpointCache {
        public static Position position;
        public static Vector2 velocity;
        public static double angularVelocity;
    }

    @Config
    public static class HeadingPID {
        public static double P = 0.02;
        public static double I = 0;
        public static double D = 0.001;
    }

    @Config
    public static class SwervePID {
        public static double P = 0.01;
        public static double I = 0;
        public static double D = 0;
    }

    public SwerveDrive(Position startPosition, boolean xPattern) {
        this.startPosition = startPosition;
        this.headingController = new PID(0, 0, 0);
        this.heading = () -> 0;
        this.xPattern = xPattern;
    }

    public SwerveDrive(Position startPosition, DoubleSupplier h, boolean xPattern) {
        this.startPosition = startPosition;
        this.headingController = new PID(0, 0, 0);
        this.heading = h;
        this.xPattern = xPattern;
    }

    @Override
    public void loadHardware(HardwareMap hardwareMap) {

        AbsoluteEncoder rightFrontAnalog = new AbsoluteEncoder(hardwareMap, Zeroing.Names.rightFrontAnalog);
        AbsoluteEncoder leftFrontAnalog = new AbsoluteEncoder(hardwareMap, Zeroing.Names.leftFrontAnalog);
        AbsoluteEncoder rightRearAnalog = new AbsoluteEncoder(hardwareMap, Zeroing.Names.rightRearAnalog);
        AbsoluteEncoder leftRearAnalog = new AbsoluteEncoder(hardwareMap, Zeroing.Names.leftRearAnalog);

        rightFrontAnalog.read();
        leftFrontAnalog.read();
        rightRearAnalog.read();
        leftRearAnalog.read();

        this.pinpoint = hardwareMap.get(GoBildaPinpointDriver.class, "pinpoint");
        this.pinpoint.setEncoderDirections(GoBildaPinpointDriver.EncoderDirection.FORWARD, GoBildaPinpointDriver.EncoderDirection.REVERSED);
        this.pinpoint.setOffsets(134.857, 0, DistanceUnit.MM);
        this.pinpoint.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);

        PinpointCache.position = new Position(startPosition.position.x(), startPosition.position.y(), startPosition.heading);
        PinpointCache.velocity = Vector2.cartesian(0, 0);
        PinpointCache.angularVelocity = 0;
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

        this.rightFront = new SwervePod(rightFront, new LionCRServo(hardwareMap, ServoConstants.Names.rightFront), Vector2.cartesian(1, 1), Zeroing.podAngle(rightFrontAnalog.position(), ConstantsStorage.get("rf", 0.0)), xPattern);
        this.leftFront = new SwervePod(leftFront, new LionCRServo(hardwareMap, ServoConstants.Names.leftFront), Vector2.cartesian(-1, 1), Zeroing.podAngle(leftFrontAnalog.position(), ConstantsStorage.get("lf", 0.0)), xPattern);
        this.rightRear = new SwervePod(rightRear, new LionCRServo(hardwareMap, ServoConstants.Names.rightRear), Vector2.cartesian(1, -1), Zeroing.podAngle(rightRearAnalog.position(), ConstantsStorage.get("rr", 0.0)), xPattern);
        this.leftRear = new SwervePod(leftRear, new LionCRServo(hardwareMap, ServoConstants.Names.leftRear), Vector2.cartesian(-1, -1), Zeroing.podAngle(leftRearAnalog.position(), ConstantsStorage.get("lr", 0.0)), xPattern);
    }

    @Override
    public void init() {
        this.pinpoint.resetPosAndIMU();
        sleep(300);
        this.pinpoint.setPosition(new Position(
                startPosition.position.y(),
                -startPosition.position.x(),
                startPosition.heading
        ).pose());
        this.targetHeading = startPosition.heading;
        this.turning = false;
        this.driveVector = Vector2.cartesian(0, 0);
    }

    @Override
    public void update(Telemetry telemetry, boolean useTelemetry) {

        this.headingController.setConstants(
            HeadingPID.P,
            HeadingPID.I,
            HeadingPID.D
        );

        this.pinpoint.update();

        Pose2D position = this.pinpoint.getPosition();
        PinpointCache.position.update(
                -position.getY(DistanceUnit.MM),
                position.getX(DistanceUnit.MM),
                position.getHeading(AngleUnit.DEGREES)
        );

        PinpointCache.angularVelocity = this.pinpoint.getHeadingVelocity(UnnormalizedAngleUnit.DEGREES);
        PinpointCache.velocity.update(
                -pinpoint.getVelY(DistanceUnit.MM),
                pinpoint.getVelX(DistanceUnit.MM)
        );

        double error = angleDifference(this.targetHeading, PinpointCache.position.heading);
        double h = heading.getAsDouble();
        double response = Math.min(0.5, Math.max(-0.5, this.headingController.calculate(error, 0)));

        response *= PinpointCache.velocity.magnitude() * -0.0004 + 1.0;
        if (PinpointCache.velocity.magnitude() < 500 && this.driveVector.magnitude() > 0.5) response *= 0.5;

        if (h != 0 || turning) {
            this.turning = true;
            this.targetHeading = PinpointCache.position.heading;

            if (Math.abs(PinpointCache.angularVelocity) < 5 && h == 0) {
                this.turning = false;
            }
        } else {
            h = response;
        }

        double a = this.rightFront.update(driveVector, h);
        double b = this.leftFront.update(driveVector, h);
        double c = this.rightRear.update(driveVector, h);
        double d = this.leftRear.update(driveVector, h);

        double maximum = Math.max(Math.max(Math.abs(a), Math.abs(b)), Math.max(Math.abs(c), Math.abs(d)));

        if (maximum > 1) {
            a /= maximum;
            b /= maximum;
            c /= maximum;
            d /= maximum;
        }

        rightFront.set(a);
        leftFront.set(b);
        rightRear.set(c);
        leftRear.set(d);

        if (useTelemetry) {
            telemetry.addData("X POSITION", SwerveDrive.PinpointCache.position.position.x());
            telemetry.addData("Y POSITION", SwerveDrive.PinpointCache.position.position.y());
            telemetry.addData("HEADING", PinpointCache.position.heading);
            telemetry.addData("HEADING START", startPosition.heading);
        }
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

    public void setTargetHeading(double newHeading) {
        this.targetHeading = newHeading;
    }
    public void setTargetVector(Vector2 vector) {
        this.driveVector = vector;
    }

    public void relocalise() {
        this.pinpoint.setPosition(startPosition.pose());
    }

    public void relocaliseTo(Position position) {
        this.pinpoint.setPosition(position.pose());
    }
}