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
    private boolean turning;

    private final boolean xPattern;

    private double omegaCommand = 0;
    private double filteredHeadingResponse = 0;

    private final boolean yawCorrection;

    @Config
    public static class HeadingPID {

        public static double low = 0.5;
        public static double limit = 1;

        public static double scale = 0;
        public static double offset = 0.01;
        public static double dScale = 0;
        public static double dOffset = 0.0005;
    }

    @Config
    public static class SwervePID {
        public static double P = 0.006;
        public static double D = 0.0;
        public static double kS = 0.08;

        public static double deadband = 8;
        public static double limitband = 10;
        public static double limit = 0.75;

        public static double latency = 0.03;
    }

    @Config
    public static class HeadingFilter {
        public static double smoothing = 0.2;
    }

    public static class PinpointCache {
        public static Position position;
        public static Vector2 velocity;
        public static double angularVelocity;
    }

    public SwerveDrive(Position startPosition, boolean xPattern, boolean yawCorrection) {
        this.startPosition = startPosition;
        this.heading = () -> 0;
        this.xPattern = xPattern;
        this.yawCorrection = yawCorrection;
    }

    public SwerveDrive(Position startPosition, DoubleSupplier h, boolean xPattern, boolean yawCorrection) {
        this.startPosition = startPosition;
        this.heading = h;
        this.xPattern = xPattern;
        this.yawCorrection = yawCorrection;
    }

    @Override
    public void loadHardware(HardwareMap hardwareMap) {

        if (PinpointCache.position != null) {
            this.startPosition = PinpointCache.position;
        }

        AbsoluteEncoder rightFrontAnalog = new AbsoluteEncoder(hardwareMap, Zeroing.Names.rightFrontAnalog);
        AbsoluteEncoder leftFrontAnalog = new AbsoluteEncoder(hardwareMap, Zeroing.Names.leftFrontAnalog);
        AbsoluteEncoder rightRearAnalog = new AbsoluteEncoder(hardwareMap, Zeroing.Names.rightRearAnalog);
        AbsoluteEncoder leftRearAnalog = new AbsoluteEncoder(hardwareMap, Zeroing.Names.leftRearAnalog);

        rightFrontAnalog.read();
        leftFrontAnalog.read();
        rightRearAnalog.read();
        leftRearAnalog.read();

        pinpoint = hardwareMap.get(GoBildaPinpointDriver.class, "pinpoint");
        pinpoint.setEncoderDirections(
                GoBildaPinpointDriver.EncoderDirection.FORWARD,
                GoBildaPinpointDriver.EncoderDirection.REVERSED
        );
        pinpoint.setOffsets(134.857, 0, DistanceUnit.MM);
        pinpoint.setEncoderResolution(
                GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD
        );

        PinpointCache.position = new Position(
                startPosition.position.x(),
                startPosition.position.y(),
                startPosition.heading
        );
        PinpointCache.velocity = Vector2.cartesian(0,0);
        PinpointCache.angularVelocity = 0;

        LionMotor rightFrontMotor = LionMotor.withEncoder(hardwareMap, MotorConstants.Names.rightFront);
        LionMotor leftFrontMotor = LionMotor.withEncoder(hardwareMap, MotorConstants.Names.leftFront);
        LionMotor rightRearMotor = LionMotor.withEncoder(hardwareMap, MotorConstants.Names.rightRear);
        LionMotor leftRearMotor = LionMotor.withEncoder(hardwareMap, MotorConstants.Names.leftRear);

        rightFrontMotor.setReversed(MotorConstants.Reversed.rf);
        leftFrontMotor.setReversed(MotorConstants.Reversed.lf);
        rightRearMotor.setReversed(MotorConstants.Reversed.rr);
        leftRearMotor.setReversed(MotorConstants.Reversed.lr);

        rightFrontMotor.setZPB(MotorConstants.ZPB.driveMotors);
        leftFrontMotor.setZPB(MotorConstants.ZPB.driveMotors);
        rightRearMotor.setZPB(MotorConstants.ZPB.driveMotors);
        leftRearMotor.setZPB(MotorConstants.ZPB.driveMotors);

        rightFrontMotor.setReverseEncoder(true);
        rightRearMotor.setReverseEncoder(true);

        rightFront = new SwervePod(
                rightFrontMotor,
                new LionCRServo(hardwareMap, ServoConstants.Names.rightFront),
                Vector2.cartesian(1,1),
                Zeroing.podAngle(rightFrontAnalog.position(), ConstantsStorage.get("rf",0.0)),
                xPattern
        );

        leftFront = new SwervePod(
                leftFrontMotor,
                new LionCRServo(hardwareMap, ServoConstants.Names.leftFront),
                Vector2.cartesian(-1,1),
                Zeroing.podAngle(leftFrontAnalog.position(), ConstantsStorage.get("lf",0.0)),
                xPattern
        );

        rightRear = new SwervePod(
                rightRearMotor,
                new LionCRServo(hardwareMap, ServoConstants.Names.rightRear),
                Vector2.cartesian(1,-1),
                Zeroing.podAngle(rightRearAnalog.position(), ConstantsStorage.get("rr",0.0)),
                xPattern
        );

        leftRear = new SwervePod(
                leftRearMotor,
                new LionCRServo(hardwareMap, ServoConstants.Names.leftRear),
                Vector2.cartesian(-1,-1),
                Zeroing.podAngle(leftRearAnalog.position(), ConstantsStorage.get("lr",0.0)),
                xPattern
        );
    }

    @Override
    public void init() {
        pinpoint.resetPosAndIMU();
        sleep(300);

        pinpoint.setPosition(new Position(
                startPosition.position.y(),
                -startPosition.position.x(),
                startPosition.heading
        ).pose());

        targetHeading = startPosition.heading;
        driveVector = Vector2.cartesian(0,0);
        turning = false;
        omegaCommand = 0;
        filteredHeadingResponse = 0;
    }

    @Override
    public void update(Telemetry telemetry, boolean useTelemetry) {

        pinpoint.update();
        Pose2D pos = pinpoint.getPosition();

        PinpointCache.position.update(
                -pos.getY(DistanceUnit.MM),
                pos.getX(DistanceUnit.MM),
                pos.getHeading(AngleUnit.DEGREES)
        );

        PinpointCache.angularVelocity =
                pinpoint.getHeadingVelocity(UnnormalizedAngleUnit.DEGREES);

        PinpointCache.velocity.update(
                -pinpoint.getVelY(DistanceUnit.MM),
                pinpoint.getVelX(DistanceUnit.MM)
        );

        double driverTurn = heading.getAsDouble() * 0.75;
        double headingNow = PinpointCache.position.heading;

        double error = angleDifference(headingNow, targetHeading);

        if (Math.abs(error) < 4) error = 0;

        double velocity = PinpointCache.velocity.magnitude();
        double hP = velocity * HeadingPID.scale + HeadingPID.offset;
        double hD = hP * HeadingPID.dScale + HeadingPID.dOffset;

        // Adaptive proportional: smaller P at low angular velocity
        double velocityFactor = Math.max(Math.min(1.0, Math.abs(PinpointCache.angularVelocity) / 50.0), HeadingPID.low);
        double adaptiveP = hP * velocityFactor;

        double rawResponse = adaptiveP * error + hD * PinpointCache.angularVelocity;

        // Low-pass smoothing
        filteredHeadingResponse += HeadingFilter.smoothing * (rawResponse - filteredHeadingResponse);

        double response = filteredHeadingResponse;
        response = Math.max(-HeadingPID.limit, Math.min(HeadingPID.limit, response));

        if (!yawCorrection) response = 0;

        // Deadband
        double turnInput = driverTurn;
        if (Math.abs(turnInput) < 0.05) turnInput = 0;

        if (turnInput != 0) {
            turning = true;
            targetHeading = headingNow;
        } else if (turning) {
            if (Math.abs(PinpointCache.angularVelocity) < 25) {
                turning = false;
                targetHeading = headingNow;
            }
        }

        double h;
        if (turning) {
            // Direct response for snappy rotation
            omegaCommand = turnInput;
            h = omegaCommand;
        } else {
            omegaCommand = response;
            h = response;
        }

        double a = rightFront.update(driveVector, h);
        double b = leftFront.update(driveVector, h);
        double c = rightRear.update(driveVector, h);
        double d = leftRear.update(driveVector, h);

        double max = Math.max(Math.max(Math.abs(a), Math.abs(b)), Math.max(Math.abs(c), Math.abs(d)));
        if (max > 1) {
            a /= max; b /= max; c /= max; d /= max;
        }

        rightFront.set(a);
        leftFront.set(b);
        rightRear.set(c);
        leftRear.set(d);

        if (useTelemetry) {
            telemetry.addData("X POSITION", PinpointCache.position.position.x());
            telemetry.addData("Y POSITION", PinpointCache.position.position.y());
            telemetry.addData("HEADING", PinpointCache.position.heading);
            telemetry.addData("TARGET HEADING", targetHeading);
            telemetry.addData("OMEGA CMD", omegaCommand);
            telemetry.addData("ANGULAR VELOCITY", PinpointCache.angularVelocity);

            telemetry.addData("frontRightTarget", rightFront.targetAngle);
            telemetry.addData("frontRightActual", rightFront.currentAngle);
            telemetry.addData("frontLeftTarget", leftFront.targetAngle);
            telemetry.addData("frontLeftActual", leftFront.currentAngle);
            telemetry.addData("rearRightTarget", rightRear.targetAngle);
            telemetry.addData("rearRightActual", rightRear.currentAngle);
            telemetry.addData("rearLeftTarget", leftRear.targetAngle);
            telemetry.addData("rearLeftActual", leftRear.currentAngle);
        }
    }

    public static double angleDifference(double target, double current) {
        double delta = target - current;
        while (delta > 180) delta -= 360;
        while (delta < -180) delta += 360;
        return delta;
    }

    public void setTargetHeading(double newHeading) { targetHeading = newHeading; }
    public void setTargetVector(Vector2 vector) { driveVector = vector; }
    public void relocalise() {
        pinpoint.setPosition(new Position(
                startPosition.position.y(),
                -startPosition.position.x(),
                startPosition.heading
        ).pose());
        this.targetHeading = startPosition.heading;
    }
    public void relocaliseTo(Position position) {
        this.targetHeading = position.heading;
        pinpoint.setPosition(position.pose());
    }

    public void setXPattern(boolean allowXPattern) {
        this.leftFront.setXPattern(allowXPattern);
        this.rightFront.setXPattern(allowXPattern);
        this.leftRear.setXPattern(allowXPattern);
        this.rightRear.setXPattern(allowXPattern);
    }

    public void clear() {
        PinpointCache.position = null;
        PinpointCache.velocity = null;
        PinpointCache.angularVelocity = 0;
    }

    public void bumpRight() {
        this.targetHeading = this.targetHeading - 45;
        while (this.targetHeading >=  180) this.targetHeading -= 360;
        while (this.targetHeading <= -180) this.targetHeading += 360;
    }

    public void bumpLeft() {
        this.targetHeading = this.targetHeading + 45;
        while (this.targetHeading >=  180) this.targetHeading -= 360;
        while (this.targetHeading <= -180) this.targetHeading += 360;
    }
}
