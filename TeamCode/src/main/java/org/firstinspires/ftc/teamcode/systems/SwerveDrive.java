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
import org.firstinspires.ftc.teamcode.lioncore.hardware.LionMotor;
import org.firstinspires.ftc.teamcode.lioncore.hardware.LionServo;
import org.firstinspires.ftc.teamcode.lioncore.math.types.Position;
import org.firstinspires.ftc.teamcode.lioncore.math.types.Vector2;
import org.firstinspires.ftc.teamcode.lioncore.systems.SystemBase;
import org.firstinspires.ftc.teamcode.parameters.MotorConstants;
import org.firstinspires.ftc.teamcode.parameters.ServoConstants;
import java.util.function.DoubleSupplier;

public class SwerveDrive extends SystemBase {

    @Config
    public static class HeadingControl {
        public static double kP = 0.012;
        public static double kD = 0.0015;
        public static double maxOutput = 0.6;
        public static double errorDeadband = 2.5;
        public static double driverScale = 0.70;
        public static double driverDeadband = 0.05;
        public static double settleOmega = 20.0;
    }

    public static class PinpointCache {
        public static Position position;
        public static Vector2  velocity;
        public static double   angularVelocity;
    }

    private GoBildaPinpointDriver pinpoint;
    private SwervePod rightFront, leftFront, rightRear, leftRear;

    // Config
    private final Position       startPosition;
    private final DoubleSupplier headingInput;
    private       boolean        oPattern;
    private final boolean        yawCorrection;

    // Control
    private Vector2 driveVector   = Vector2.cartesian(0.0, 0.0);
    private double  targetHeading = 0.0;
    private boolean turning       = false;
    private double  omegaCommand  = 0.0;
    private Vector2 directionPreload = Vector2.cartesian(0, 0);

    public SwerveDrive(Position startPosition, boolean oPattern, boolean yawCorrection) {
        this.startPosition = startPosition;
        this.headingInput  = () -> 0.0;
        this.oPattern      = oPattern;
        this.yawCorrection = yawCorrection;
    }

    public SwerveDrive(Position startPosition, DoubleSupplier headingInput,
                       boolean oPattern, boolean yawCorrection) {
        this.startPosition = startPosition;
        this.headingInput  = headingInput;
        this.oPattern      = oPattern;
        this.yawCorrection = yawCorrection;
    }

    @Override
    public void loadHardware(HardwareMap hardwareMap) {

        pinpoint = hardwareMap.get(GoBildaPinpointDriver.class, "pinpoint");
        pinpoint.setEncoderDirections(
                GoBildaPinpointDriver.EncoderDirection.FORWARD,
                GoBildaPinpointDriver.EncoderDirection.REVERSED);
        pinpoint.setOffsets(134.857, 0, DistanceUnit.MM);
        pinpoint.setEncoderResolution(
                GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);

        Position seed = (PinpointCache.position != null)
                ? PinpointCache.position : startPosition;
        PinpointCache.position        = new Position(seed.position.x(),
                seed.position.y(),
                seed.heading);
        PinpointCache.velocity        = Vector2.cartesian(0.0, 0.0);
        PinpointCache.angularVelocity = 0.0;

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

        LionServo rfServo = LionServo.single(hardwareMap, ServoConstants.Names.rightFront, 0.5);
        LionServo lfServo = LionServo.single(hardwareMap, ServoConstants.Names.leftFront,  0.5);
        LionServo rrServo = LionServo.single(hardwareMap, ServoConstants.Names.rightRear,  0.5);
        LionServo lrServo = LionServo.single(hardwareMap, ServoConstants.Names.leftRear,   0.5);

        rightFront = new SwervePod(rfMotor, rfServo, Vector2.cartesian( 1.0,  1.0), -3.0);
        leftFront  = new SwervePod(lfMotor, lfServo, Vector2.cartesian(-1.0,  1.0), 5.0);
        rightRear  = new SwervePod(rrMotor, rrServo, Vector2.cartesian( 1.0, -1.0), -3.0);
        leftRear   = new SwervePod(lrMotor, lrServo, Vector2.cartesian(-1.0, -1.0), 0.0);
    }

    @Override
    public void init() {
        pinpoint.resetPosAndIMU();
        sleep(300);

        pinpoint.setPosition(new Position(
                startPosition.position.y(),
                -startPosition.position.x(),
                startPosition.heading).pose());

        targetHeading = startPosition.heading;
        driveVector   = Vector2.cartesian(0.0, 0.0);
        omegaCommand  = 0.0;
        turning       = false;
    }

    @Override
    public void update(Telemetry telemetry, boolean useTelemetry) {

        pinpoint.update();
        Pose2D pose = pinpoint.getPosition();

        PinpointCache.position.update(
                -pose.getY(DistanceUnit.MM),
                pose.getX(DistanceUnit.MM),
                pose.getHeading(AngleUnit.DEGREES));
        PinpointCache.angularVelocity =
                pinpoint.getHeadingVelocity(UnnormalizedAngleUnit.DEGREES);
        PinpointCache.velocity.update(
                -pinpoint.getVelY(DistanceUnit.MM),
                pinpoint.getVelX(DistanceUnit.MM));

        final double heading = PinpointCache.position.heading;
        final double omega   = PinpointCache.angularVelocity;

        double rawTurn    = -headingInput.getAsDouble();
        double driverTurn = (Math.abs(rawTurn) < HeadingControl.driverDeadband)
                ? 0.0 : rawTurn * HeadingControl.driverScale;

        if (driverTurn != 0.0) {
            turning       = true;
            targetHeading = heading;
            omegaCommand  = driverTurn;

        } else if (turning) {
            if (Math.abs(omega) < HeadingControl.settleOmega) {
                turning       = false;
                targetHeading = heading;
                omegaCommand  = 0.0;
            } else {
                omegaCommand = -HeadingControl.kD * omega;
            }

        } else {
            double error = wrapDeg(targetHeading - heading);

            if (Math.abs(error) < HeadingControl.errorDeadband) {
                omegaCommand = 0.0;
            } else {
                omegaCommand = HeadingControl.kP * error - HeadingControl.kD * omega;
                omegaCommand = Math.max(-HeadingControl.maxOutput,
                        Math.min( HeadingControl.maxOutput, omegaCommand));
            }
        }

        if (!yawCorrection) {
            omegaCommand = (driverTurn != 0.0) ? driverTurn : 0.0;
        }

        boolean idle = driveVector.magnitude() < 0.05 && Math.abs(omegaCommand) < 0.05;

        double a, b, c, d;
        if (idle) {
            if (this.directionPreload.magnitude() == 0) {
                a = rightFront.updateIdle(oPattern);
                b = leftFront.updateIdle(oPattern);
                c = rightRear.updateIdle(oPattern);
                d = leftRear.updateIdle(oPattern);
            } else {
                a = rightFront.updatePreload(directionPreload);
                b = leftFront.updatePreload(directionPreload);
                c = rightRear.updatePreload(directionPreload);
                d = leftRear.updatePreload(directionPreload);
            }
        } else {
            a = rightFront.update(driveVector, -omegaCommand);
            b = leftFront .update(driveVector, -omegaCommand);
            c = rightRear .update(driveVector, -omegaCommand);
            d = leftRear  .update(driveVector, -omegaCommand);
        }

        double maxPow = Math.max(Math.max(Math.abs(a), Math.abs(b)),
                Math.max(Math.abs(c), Math.abs(d)));
        if (maxPow > 1.0) { a /= maxPow; b /= maxPow; c /= maxPow; d /= maxPow; }

        rightFront.applyPower(a);
        leftFront .applyPower(b);
        rightRear .applyPower(c);
        leftRear  .applyPower(d);

        if (useTelemetry) {
            String state = turning
                    ? (Math.abs(omega) < HeadingControl.settleOmega ? "SETTLING" : "TURNING")
                    : "HOLDING";
            telemetry.addData("── Heading ──",        "");
            telemetry.addData("  State",              state);
            telemetry.addData("  Heading",            String.format("%.1f°", heading));
            telemetry.addData("  Target",             String.format("%.1f°", targetHeading));
            telemetry.addData("  Error",              String.format("%.1f°", wrapDeg(targetHeading - heading)));
            telemetry.addData("  ω measured",         String.format("%.1f °/s", omega));
            telemetry.addData("  ω command",          String.format("%.3f",     omegaCommand));
            telemetry.addData("── Position ──",       "");
            telemetry.addData("  X",                  String.format("%.1f mm", PinpointCache.position.position.x()));
            telemetry.addData("  Y",                  String.format("%.1f mm", PinpointCache.position.position.y()));
            telemetry.addData("── Pods (commanded) ──", "");
            telemetry.addData("  RF",                 String.format("%.1f°", rightFront.currentAngle));
            telemetry.addData("  LF",                 String.format("%.1f°", leftFront .currentAngle));
            telemetry.addData("  RR",                 String.format("%.1f°", rightRear .currentAngle));
            telemetry.addData("  LR",                 String.format("%.1f°", leftRear  .currentAngle));
        }
    }

    public void setTargetVector(Vector2 vector) {
        driveVector = vector;
    }

    public void setTargetHeading(double degrees) {
        targetHeading = degrees;
        turning       = false;
    }

    public void setOPattern(boolean enabled) {
        oPattern = enabled;
    }

    public void relocalise() {
        pinpoint.setPosition(new Position(
                startPosition.position.y(),
                -startPosition.position.x(),
                startPosition.heading).pose());
        targetHeading = startPosition.heading;
        turning       = false;
    }

    public void relocaliseTo(Position position) {
        pinpoint.setPosition(position.pose());
        targetHeading = position.heading;
        turning       = false;
    }

    public void bumpRight() {
        targetHeading = wrapDeg(targetHeading - 45.0);
        turning       = false;
    }

    public void bumpLeft() {
        targetHeading = wrapDeg(targetHeading + 45.0);
        turning       = false;
    }

    public void clear() {
        PinpointCache.position        = null;
        PinpointCache.velocity        = null;
        PinpointCache.angularVelocity = 0.0;
    }

    private static double wrapDeg(double a) {
        a = ((a % 360.0) + 360.0) % 360.0;
        if (a > 180.0) a -= 360.0;
        return a;
    }
}
