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

    private Position       startPosition;
    private Vector2        driveVector;
    private DoubleSupplier heading;

    private double  targetHeading;
    private boolean turning;
    private boolean oPattern;

    private double omegaCommand            = 0;
    private double filteredHeadingResponse = 0;
    private double headingIntegral         = 0;
    private double filteredOmega           = 0;

    private final boolean yawCorrection;

    @Config
    public static class HeadingControl {
        /** Proportional gain on heading error (deg → output). */
        public static double kP = 0.01;
        /** Derivative gain on measured angular velocity (deg/s → output). Keep positive. */
        public static double kD = 0.0001;
        /** Integral gain on accumulated heading error. Start small (~0.0001). */
        public static double kI = 0.0001;
        /** Heading error (deg) below which no correction is applied. */
        public static double errorDeadband = 4.0;
        /** Maximum yaw-correction output magnitude. */
        public static double outputLimit = 1.0;
        /** Low-pass coefficient for the heading response [0 = frozen … 1 = fully raw]. */
        public static double filterSmoothing = 0.2;
        /** EMA coefficient for the angular velocity derivative term [0 = frozen … 1 = fully raw]. */
        public static double omegaFilter = 0.25;
        /** Driver turn input scale factor. */
        public static double driverScale = 0.75;
        /** Driver turn inputs below this magnitude are treated as zero. */
        public static double driverDeadband = 0.05;
        /** Angular velocity (deg/s) below which the robot is considered settled after releasing the turn stick. */
        public static double turnSettleOmega = 25.0;
        /** Maximum integrator contribution magnitude, prevents windup. */
        public static double integralLimit = 0.2;
        /** Heading error (deg) window within which the integrator accumulates. */
        public static double integralWindow = 15.0;
        /** Latency compensation. */
        public static double latency = 0.001;
    }

    @Config
    public static class PodControl {
        /** Proportional gain on steering angle error. */
        public static double kP = 0.007;
        /** Derivative gain on measured pod angular velocity (deg/s → output). Keep positive. */
        public static double kD = 0.0;
        /** Velocity feedforward gain (setpoint deg/s → output). Tune until pod tracks rotating targets without lag. */
        public static double kV = 0.001;
        /** Static-friction feedforward, scaled by runtime voltage compensation. */
        public static double kS = 0.02;
        /** Transition width (deg) for the soft-sign kS ramp. Smaller = sharper transition. */
        public static double kSTransition = 5.0;
        /** EMA coefficient for pod angular velocity estimate [0 = frozen … 1 = fully raw]. */
        public static double velocityFilter = 0.3;
        /** Steering error (deg) below which servo output is cut to zero. */
        public static double errorDeadband = 10.0;
        /** Steering error (deg) below which output is clamped to ±outputLimit. */
        public static double limitband = 15.0;
        /** Maximum servo output when inside the limitband. */
        public static double outputLimit = 0.75;
        /** Estimated sensor/actuator latency (s) for predictive angle compensation. */
        public static double latency = 0.001;
    }

    public static class PinpointCache {
        public static Position position;
        public static Vector2  velocity;
        public static double   angularVelocity;
    }

    public SwerveDrive(Position startPosition, boolean oPattern, boolean yawCorrection) {
        this.startPosition = startPosition;
        this.heading       = () -> 0;
        this.oPattern      = oPattern;
        this.yawCorrection = yawCorrection;
    }

    public SwerveDrive(Position startPosition, DoubleSupplier heading,
                       boolean oPattern, boolean yawCorrection) {
        this.startPosition = startPosition;
        this.heading       = heading;
        this.oPattern      = oPattern;
        this.yawCorrection = yawCorrection;
    }

    @Override
    public void loadHardware(HardwareMap hardwareMap) {

        if (PinpointCache.position != null) startPosition = PinpointCache.position;

        AbsoluteEncoder rfEnc = new AbsoluteEncoder(hardwareMap, Zeroing.Names.rightFrontAnalog);
        AbsoluteEncoder lfEnc = new AbsoluteEncoder(hardwareMap, Zeroing.Names.leftFrontAnalog);
        AbsoluteEncoder rrEnc = new AbsoluteEncoder(hardwareMap, Zeroing.Names.rightRearAnalog);
        AbsoluteEncoder lrEnc = new AbsoluteEncoder(hardwareMap, Zeroing.Names.leftRearAnalog);
        rfEnc.read(); lfEnc.read(); rrEnc.read(); lrEnc.read();

        pinpoint = hardwareMap.get(GoBildaPinpointDriver.class, "pinpoint");
        pinpoint.setEncoderDirections(
                GoBildaPinpointDriver.EncoderDirection.FORWARD,
                GoBildaPinpointDriver.EncoderDirection.REVERSED);
        pinpoint.setOffsets(134.857, 0, DistanceUnit.MM);
        pinpoint.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);

        PinpointCache.position        = new Position(startPosition.position.x(),
                startPosition.position.y(),
                startPosition.heading);
        PinpointCache.velocity        = Vector2.cartesian(0, 0);
        PinpointCache.angularVelocity = 0;

        LionMotor rfMotor = LionMotor.withEncoder(hardwareMap, MotorConstants.Names.rightFront);
        LionMotor lfMotor = LionMotor.withEncoder(hardwareMap, MotorConstants.Names.leftFront);
        LionMotor rrMotor = LionMotor.withEncoder(hardwareMap, MotorConstants.Names.rightRear);
        LionMotor lrMotor = LionMotor.withEncoder(hardwareMap, MotorConstants.Names.leftRear);

        rfMotor.setReversed(MotorConstants.Reversed.rf);
        lfMotor.setReversed(MotorConstants.Reversed.lf);
        rrMotor.setReversed(MotorConstants.Reversed.rr);
        lrMotor.setReversed(MotorConstants.Reversed.lr);

        rfMotor.setZPB(MotorConstants.ZPB.driveMotors);
        lfMotor.setZPB(MotorConstants.ZPB.driveMotors);
        rrMotor.setZPB(MotorConstants.ZPB.driveMotors);
        lrMotor.setZPB(MotorConstants.ZPB.driveMotors);

        rfMotor.setReverseEncoder(true);
        rrMotor.setReverseEncoder(true);

        rightFront = new SwervePod(rfMotor,
                new LionCRServo(hardwareMap, ServoConstants.Names.rightFront),
                Vector2.cartesian( 1,  1),
                Zeroing.podAngle(rfEnc.position(), ConstantsStorage.get("rf", 0.0)));

        leftFront = new SwervePod(lfMotor,
                new LionCRServo(hardwareMap, ServoConstants.Names.leftFront),
                Vector2.cartesian(-1,  1),
                Zeroing.podAngle(lfEnc.position(), ConstantsStorage.get("lf", 0.0)));

        rightRear = new SwervePod(rrMotor,
                new LionCRServo(hardwareMap, ServoConstants.Names.rightRear),
                Vector2.cartesian( 1, -1),
                Zeroing.podAngle(rrEnc.position(), ConstantsStorage.get("rr", 0.0)));

        leftRear = new SwervePod(lrMotor,
                new LionCRServo(hardwareMap, ServoConstants.Names.leftRear),
                Vector2.cartesian(-1, -1),
                Zeroing.podAngle(lrEnc.position(), ConstantsStorage.get("lr", 0.0)));
    }

    @Override
    public void init() {
        pinpoint.resetPosAndIMU();
        sleep(300);

        pinpoint.setPosition(new Position(
                startPosition.position.y(),
                -startPosition.position.x(),
                startPosition.heading).pose());

        targetHeading           = startPosition.heading;
        driveVector             = Vector2.cartesian(0, 0);
        turning                 = false;
        omegaCommand            = 0;
        filteredHeadingResponse = 0;
        headingIntegral         = 0;
        filteredOmega           = 0;
    }

    @Override
    public void update(Telemetry telemetry, boolean useTelemetry) {

        pinpoint.update();
        Pose2D pos = pinpoint.getPosition();

        PinpointCache.position.update(
                -pos.getY(DistanceUnit.MM),
                pos.getX(DistanceUnit.MM),
                pos.getHeading(AngleUnit.DEGREES));

        PinpointCache.angularVelocity =
                pinpoint.getHeadingVelocity(UnnormalizedAngleUnit.DEGREES);

        PinpointCache.velocity.update(
                -pinpoint.getVelY(DistanceUnit.MM),
                pinpoint.getVelX(DistanceUnit.MM));

        double driverTurn = heading.getAsDouble();
        if (Math.abs(driverTurn) < HeadingControl.driverDeadband) driverTurn = 0;
        else                                                       driverTurn *= HeadingControl.driverScale;

        double headingNow = PinpointCache.position.heading + HeadingControl.latency * PinpointCache.angularVelocity;
        double error      = angleDifference(headingNow, targetHeading);
        if (Math.abs(error) < HeadingControl.errorDeadband) error = 0;

        filteredOmega += HeadingControl.omegaFilter
                * (PinpointCache.angularVelocity - filteredOmega);

        if (!turning && Math.abs(error) < HeadingControl.integralWindow) {
            headingIntegral += error * HeadingControl.kI;
            headingIntegral  = Math.max(-HeadingControl.integralLimit,
                    Math.min( HeadingControl.integralLimit, headingIntegral));
        } else {
            headingIntegral *= 0.9;
        }

        double rawResponse = HeadingControl.kP * error
                + headingIntegral
                - HeadingControl.kD * filteredOmega;

        filteredHeadingResponse +=
                HeadingControl.filterSmoothing * (rawResponse - filteredHeadingResponse);
        double response = Math.max(-HeadingControl.outputLimit,
                Math.min( HeadingControl.outputLimit, filteredHeadingResponse));

        if (!yawCorrection) response = 0;

        if (driverTurn != 0) {
            turning       = true;
            targetHeading = headingNow;
        } else if (turning
                && Math.abs(PinpointCache.angularVelocity) < HeadingControl.turnSettleOmega) {
            turning         = false;
            targetHeading   = headingNow;
            headingIntegral = 0;
        }

        omegaCommand = turning ? driverTurn : response;

        boolean idle = driveVector.magnitude() < 0.1 && Math.abs(omegaCommand) < 0.1;

        double a, b, c, d;
        if (idle) {
            a = rightFront.updateIdle(oPattern);
            b = leftFront .updateIdle(oPattern);
            c = rightRear .updateIdle(oPattern);
            d = leftRear  .updateIdle(oPattern);
        } else {
            a = rightFront.update(driveVector, omegaCommand);
            b = leftFront .update(driveVector, omegaCommand);
            c = rightRear .update(driveVector, omegaCommand);
            d = leftRear  .update(driveVector, omegaCommand);
        }

        double maxPow = Math.max(Math.max(Math.abs(a), Math.abs(b)),
                Math.max(Math.abs(c), Math.abs(d)));
        if (maxPow > 1) { a /= maxPow; b /= maxPow; c /= maxPow; d /= maxPow; }

        rightFront.set(a);
        leftFront .set(b);
        rightRear .set(c);
        leftRear  .set(d);

        if (useTelemetry) {
            telemetry.addData("X POSITION",       PinpointCache.position.position.x());
            telemetry.addData("Y POSITION",       PinpointCache.position.position.y());
            telemetry.addData("HEADING",          PinpointCache.position.heading);
            telemetry.addData("TARGET HEADING",   targetHeading);
            telemetry.addData("OMEGA CMD",        omegaCommand);
            telemetry.addData("ANGULAR VELOCITY", PinpointCache.angularVelocity);
            telemetry.addData("HEADING INTEGRAL", headingIntegral);
            telemetry.addData("frontRight target/actual",
                    String.format("%.1f / %.1f", rightFront.targetAngle, rightFront.currentAngle));
            telemetry.addData("frontLeft  target/actual",
                    String.format("%.1f / %.1f", leftFront.targetAngle,  leftFront.currentAngle));
            telemetry.addData("rearRight  target/actual",
                    String.format("%.1f / %.1f", rightRear.targetAngle,  rightRear.currentAngle));
            telemetry.addData("rearLeft   target/actual",
                    String.format("%.1f / %.1f", leftRear.targetAngle,   leftRear.currentAngle));
        }
    }

    public static double angleDifference(double target, double current) {
        double delta = ((target - current) % 360 + 360) % 360;
        if (delta > 180) delta -= 360;
        return delta;
    }

    private static double wrapHeading(double h) {
        h = ((h % 360) + 360) % 360;
        if (h > 180) h -= 360;
        return h;
    }

    public void setTargetHeading(double newHeading) { targetHeading = newHeading; }
    public void setTargetVector(Vector2 vector)     { driveVector   = vector; }
    public void setOPattern(boolean enabled)        { this.oPattern = enabled; }

    public void relocalise() {
        pinpoint.setPosition(new Position(
                startPosition.position.y(),
                -startPosition.position.x(),
                startPosition.heading).pose());
        targetHeading   = startPosition.heading;
        headingIntegral = 0;
    }

    public void relocaliseTo(Position position) {
        targetHeading   = position.heading;
        headingIntegral = 0;
        pinpoint.setPosition(position.pose());
    }

    public void bumpRight() { targetHeading = wrapHeading(targetHeading - 45); }
    public void bumpLeft()  { targetHeading = wrapHeading(targetHeading + 45); }

    public void clear() {
        PinpointCache.position        = null;
        PinpointCache.velocity        = null;
        PinpointCache.angularVelocity = 0;
    }
}
