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

/**
 * Swerve drive system with GoBilda Pinpoint odometry and a three-state
 * heading controller.
 *
 * ── Heading controller states ──────────────────────────────────────────────
 *
 *   TURNING   Driver is actively commanding rotation.
 *             omega = scaled driver input; targetHeading tracks live heading
 *             so it is already at the right value the moment the stick releases.
 *
 *   SETTLING  Driver released the turn stick but the robot is still spinning
 *             faster than settleOmega.  A small −kD·ω braking term is applied
 *             to help the robot decelerate and lock cleanly.
 *
 *   HOLDING   Robot is settled.  A PD loop drives heading error to zero:
 *               ω_cmd = kP·error − kD·ω_measured
 *             clipped to ±maxOutput, with an errorDeadband near zero.
 *
 * ── yawCorrection flag ─────────────────────────────────────────────────────
 *
 *   When false the entire heading-hold stack is bypassed.  Driver turn input
 *   is still passed through normally; the robot simply does not self-correct.
 */
public class SwerveDrive extends SystemBase {

    // ══════════════════════════════════════════════════════════════════════════
    //  Tuning — edit via FTC Dashboard or directly here
    // ══════════════════════════════════════════════════════════════════════════

    @Config
    public static class HeadingControl {

        /**
         * Proportional gain: heading error (deg) → omega command.
         * Increase until heading snaps back firmly without oscillating.
         * Starting point for 20 Hz: ~0.012.
         */
        public static double kP = 0.012;

        /**
         * Derivative gain: measured angular velocity (deg/s) → omega command.
         * Provides damping.  Applied in both HOLDING (dampen oscillation) and
         * SETTLING (help decelerate after turning).
         * Starting point: ~0.0015.
         */
        public static double kD = 0.0015;

        /**
         * Output clamp on the heading-hold correction.
         * Prevents the heading controller from saturating the drivetrain when
         * heading error is large (e.g. after an auto relocalisaton).
         */
        public static double maxOutput = 0.6;

        /**
         * Heading error (deg) below which the HOLDING correction is zeroed.
         * Prevents buzzing/micro-corrections near the target.
         */
        public static double errorDeadband = 2.5;

        /** Scale applied to the raw driver turn-stick value. */
        public static double driverScale = 0.70;

        /** Turn-stick values below this magnitude are treated as zero. */
        public static double driverDeadband = 0.05;

        /**
         * Angular velocity (deg/s) below which the robot is considered settled
         * after the driver releases the turn stick.  The system transitions
         * SETTLING → HOLDING only when |ω| drops below this value.
         * Increase if heading locks inconsistently; decrease if it waits too long.
         */
        public static double settleOmega = 20.0;
    }

    // ══════════════════════════════════════════════════════════════════════════
    //  Odometry cache — shared read-only with other systems
    // ══════════════════════════════════════════════════════════════════════════

    public static class PinpointCache {
        public static Position position;
        public static Vector2  velocity;
        public static double   angularVelocity;
    }

    // ══════════════════════════════════════════════════════════════════════════
    //  Fields
    // ══════════════════════════════════════════════════════════════════════════

    // ── Hardware ──────────────────────────────────────────────────────────────
    private GoBildaPinpointDriver pinpoint;
    private SwervePod rightFront, leftFront, rightRear, leftRear;

    // ── Configuration (set once at construction) ──────────────────────────────
    private final Position       startPosition;
    private final DoubleSupplier headingInput;   // driver turn-stick supplier
    private       boolean        oPattern;
    private final boolean        yawCorrection;

    // ── Runtime state ─────────────────────────────────────────────────────────
    private Vector2 driveVector   = Vector2.cartesian(0.0, 0.0);
    private double  targetHeading = 0.0;   // degrees, the heading we are holding
    private boolean turning       = false; // true while in TURNING or SETTLING state
    private double  omegaCommand  = 0.0;

    // ══════════════════════════════════════════════════════════════════════════
    //  Constructors
    // ══════════════════════════════════════════════════════════════════════════

    /** Use when heading input is controlled programmatically (autonomous, etc.). */
    public SwerveDrive(Position startPosition, boolean oPattern, boolean yawCorrection) {
        this.startPosition = startPosition;
        this.headingInput  = () -> 0.0;
        this.oPattern      = oPattern;
        this.yawCorrection = yawCorrection;
    }

    /** Use when heading input comes from a driver gamepad axis. */
    public SwerveDrive(Position startPosition, DoubleSupplier headingInput,
                       boolean oPattern, boolean yawCorrection) {
        this.startPosition = startPosition;
        this.headingInput  = headingInput;
        this.oPattern      = oPattern;
        this.yawCorrection = yawCorrection;
    }

    // ══════════════════════════════════════════════════════════════════════════
    //  SystemBase lifecycle
    // ══════════════════════════════════════════════════════════════════════════

    @Override
    public void loadHardware(HardwareMap hardwareMap) {

        // ── Pinpoint odometry ─────────────────────────────────────────────────
        pinpoint = hardwareMap.get(GoBildaPinpointDriver.class, "pinpoint");
        pinpoint.setEncoderDirections(
                GoBildaPinpointDriver.EncoderDirection.FORWARD,
                GoBildaPinpointDriver.EncoderDirection.REVERSED);
        pinpoint.setOffsets(134.857, 0, DistanceUnit.MM);
        pinpoint.setEncoderResolution(
                GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);

        // Seed the cache — carry forward from a previous op-mode if available
        Position seed = (PinpointCache.position != null)
                ? PinpointCache.position : startPosition;
        PinpointCache.position        = new Position(seed.position.x(),
                seed.position.y(),
                seed.heading);
        PinpointCache.velocity        = Vector2.cartesian(0.0, 0.0);
        PinpointCache.angularVelocity = 0.0;

        // ── Drive motors ──────────────────────────────────────────────────────
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

        // ── Steering servos ───────────────────────────────────────────────────
        //    Initialised to 0.5 so pods are mechanically forward before init().
        LionServo rfServo = LionServo.single(hardwareMap, ServoConstants.Names.rightFront, 0.5);
        LionServo lfServo = LionServo.single(hardwareMap, ServoConstants.Names.leftFront,  0.5);
        LionServo rrServo = LionServo.single(hardwareMap, ServoConstants.Names.rightRear,  0.5);
        LionServo lrServo = LionServo.single(hardwareMap, ServoConstants.Names.leftRear,   0.5);

        // ── Swerve pods ───────────────────────────────────────────────────────
        //    Offset vectors (±1, ±1) define wheel positions in robot frame.
        //    x = lateral (right positive), y = longitudinal (forward positive).
        rightFront = new SwervePod(rfMotor, rfServo, Vector2.cartesian( 1.0,  1.0), -3.0);
        leftFront  = new SwervePod(lfMotor, lfServo, Vector2.cartesian(-1.0,  1.0), 5.0);
        rightRear  = new SwervePod(rrMotor, rrServo, Vector2.cartesian( 1.0, -1.0), -3.0);
        leftRear   = new SwervePod(lrMotor, lrServo, Vector2.cartesian(-1.0, -1.0), 0.0);
    }

    @Override
    public void init() {
        pinpoint.resetPosAndIMU();
        sleep(300);

        // Pinpoint uses a rotated coordinate frame: its (X, Y) = robot (-Y, X).
        pinpoint.setPosition(new Position(
                startPosition.position.y(),
                -startPosition.position.x(),
                startPosition.heading).pose());

        targetHeading = startPosition.heading;
        driveVector   = Vector2.cartesian(0.0, 0.0);
        omegaCommand  = 0.0;
        turning       = false;
    }

    // ══════════════════════════════════════════════════════════════════════════
    //  Main loop
    // ══════════════════════════════════════════════════════════════════════════

    @Override
    public void update(Telemetry telemetry, boolean useTelemetry) {

        // ── 1. Odometry ───────────────────────────────────────────────────────
        pinpoint.update();
        Pose2D pose = pinpoint.getPosition();

        // Convert from Pinpoint frame (X=forward, Y=left) to robot frame (X=right, Y=forward)
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

        // ── 2. Heading controller ─────────────────────────────────────────────
        double rawTurn    = -headingInput.getAsDouble();
        double driverTurn = (Math.abs(rawTurn) < HeadingControl.driverDeadband)
                ? 0.0 : rawTurn * HeadingControl.driverScale;

        if (driverTurn != 0.0) {
            // ── TURNING ───────────────────────────────────────────────────────
            //    Live-track heading so targetHeading is already correct the
            //    instant the driver releases the stick.
            turning       = true;
            targetHeading = heading;
            omegaCommand  = driverTurn;

        } else if (turning) {
            // ── SETTLING ──────────────────────────────────────────────────────
            //    Driver released; wait for spin to decay.
            //    Apply gentle velocity damping to help the robot stop cleanly.
            if (Math.abs(omega) < HeadingControl.settleOmega) {
                turning       = false;
                targetHeading = heading;   // lock the heading we've arrived at
                omegaCommand  = 0.0;
            } else {
                omegaCommand = -HeadingControl.kD * omega;
            }

        } else {
            // ── HOLDING ───────────────────────────────────────────────────────
            //    PD controller: drives heading error to zero.
            //      ω_cmd =  kP * error               (proportional restore)
            //             − kD * ω_measured           (derivative damping)
            double error = wrapDeg(targetHeading - heading);

            if (Math.abs(error) < HeadingControl.errorDeadband) {
                omegaCommand = 0.0;
            } else {
                omegaCommand = HeadingControl.kP * error - HeadingControl.kD * omega;
                omegaCommand = Math.max(-HeadingControl.maxOutput,
                        Math.min( HeadingControl.maxOutput, omegaCommand));
            }
        }

        // Override: if yaw correction is disabled, bypass hold entirely.
        // Driver turn still works; the robot just doesn't self-correct.
        if (!yawCorrection) {
            omegaCommand = (driverTurn != 0.0) ? driverTurn : 0.0;
        }

        // ── 3. Pod kinematics ─────────────────────────────────────────────────
        boolean idle = driveVector.magnitude() < 0.05 && Math.abs(omegaCommand) < 0.05;

        double a, b, c, d;
        if (idle) {
            a = rightFront.updateIdle(oPattern);
            b = leftFront .updateIdle(oPattern);
            c = rightRear .updateIdle(oPattern);
            d = leftRear  .updateIdle(oPattern);
        } else {
            a = rightFront.update(driveVector, -omegaCommand);
            b = leftFront .update(driveVector, -omegaCommand);
            c = rightRear .update(driveVector, -omegaCommand);
            d = leftRear  .update(driveVector, -omegaCommand);
        }

        // ── 4. Power normalisation ────────────────────────────────────────────
        //    Scale all powers down together if any exceeds 1, preserving ratios.
        double maxPow = Math.max(Math.max(Math.abs(a), Math.abs(b)),
                Math.max(Math.abs(c), Math.abs(d)));
        if (maxPow > 1.0) { a /= maxPow; b /= maxPow; c /= maxPow; d /= maxPow; }

        rightFront.applyPower(a);
        leftFront .applyPower(b);
        rightRear .applyPower(c);
        leftRear  .applyPower(d);

        // ── 5. Telemetry ──────────────────────────────────────────────────────
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

    // ══════════════════════════════════════════════════════════════════════════
    //  Public control API
    // ══════════════════════════════════════════════════════════════════════════

    /** Sets the translation demand vector for the next update call. */
    public void setTargetVector(Vector2 vector) {
        driveVector = vector;
    }

    /**
     * Programmatically commands a new target heading and immediately engages
     * HOLDING mode.  Use in autonomous routines.
     */
    public void setTargetHeading(double degrees) {
        targetHeading = degrees;
        turning       = false;
    }

    public void setOPattern(boolean enabled) {
        oPattern = enabled;
    }

    /** Resets odometry to startPosition and re-locks targetHeading to it. */
    public void relocalise() {
        pinpoint.setPosition(new Position(
                startPosition.position.y(),
                -startPosition.position.x(),
                startPosition.heading).pose());
        targetHeading = startPosition.heading;
        turning       = false;
    }

    /** Resets odometry to an arbitrary position and locks heading to it. */
    public void relocaliseTo(Position position) {
        pinpoint.setPosition(position.pose());
        targetHeading = position.heading;
        turning       = false;
    }

    /** Snaps targetHeading 45° clockwise. */
    public void bumpRight() {
        targetHeading = wrapDeg(targetHeading - 45.0);
        turning       = false;
    }

    /** Snaps targetHeading 45° counter-clockwise. */
    public void bumpLeft() {
        targetHeading = wrapDeg(targetHeading + 45.0);
        turning       = false;
    }

    /** Invalidates the PinpointCache.  Call at the end of the final op-mode
     *  that should NOT carry odometry forward. */
    public void clear() {
        PinpointCache.position        = null;
        PinpointCache.velocity        = null;
        PinpointCache.angularVelocity = 0.0;
    }

    // ══════════════════════════════════════════════════════════════════════════
    //  Static utilities
    // ══════════════════════════════════════════════════════════════════════════

    /** Wraps {@code a} to (−180, +180]. */
    private static double wrapDeg(double a) {
        a = ((a % 360.0) + 360.0) % 360.0;
        if (a > 180.0) a -= 360.0;
        return a;
    }

    /**
     * Returns {@code (target − current)} wrapped to (−180, +180].
     * Kept as a named public helper for use by other systems (auto, etc.).
     */
    public static double angleDifference(double target, double current) {
        return ((target - current) % 360.0 + 360.0) % 360.0 > 180.0
                ? ((target - current) % 360.0 + 360.0) % 360.0 - 360.0
                : ((target - current) % 360.0 + 360.0) % 360.0;
    }
}
