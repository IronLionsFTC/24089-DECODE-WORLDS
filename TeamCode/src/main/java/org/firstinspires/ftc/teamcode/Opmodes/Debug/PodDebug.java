package org.firstinspires.ftc.teamcode.Opmodes.Debug;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.lioncore.hardware.AbsoluteEncoder;
import org.firstinspires.ftc.teamcode.lioncore.hardware.LionCRServo;
import org.firstinspires.ftc.teamcode.lioncore.hardware.LionMotor;
import org.firstinspires.ftc.teamcode.lioncore.math.pid.PID;
import org.firstinspires.ftc.teamcode.lioncore.system.ConstantsStorage;
import org.firstinspires.ftc.teamcode.parameters.MotorConstants;
import org.firstinspires.ftc.teamcode.parameters.ServoConstants;
import org.firstinspires.ftc.teamcode.parameters.Zeroing;
import org.firstinspires.ftc.teamcode.systems.SwerveDrive;
import org.firstinspires.ftc.teamcode.lioncore.tasks.TaskOpMode;

@TeleOp(name = "Swerve Angle Align (Manual PID)")
public class PodDebug extends LinearOpMode {

    @Config
    public static class Targets {
        public static double rf = 0;
        public static double lf = 0;
        public static double rr = 0;
        public static double lr = 0;
    }

    private static double wrapDeg(double angle) {
        angle %= 360;
        if (angle <= -180) angle += 360;
        if (angle > 180) angle -= 360;
        return angle;
    }

    private static double closestEquivalent(double target, double reference) {
        return reference + wrapDeg(target - reference);
    }

    @Override
    public void runOpMode() {

        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        // Absolute encoders
        AbsoluteEncoder rfAbs = new AbsoluteEncoder(hardwareMap, Zeroing.Names.rightFrontAnalog);
        AbsoluteEncoder lfAbs = new AbsoluteEncoder(hardwareMap, Zeroing.Names.leftFrontAnalog);
        AbsoluteEncoder rrAbs = new AbsoluteEncoder(hardwareMap, Zeroing.Names.rightRearAnalog);
        AbsoluteEncoder lrAbs = new AbsoluteEncoder(hardwareMap, Zeroing.Names.leftRearAnalog);

        rfAbs.read(); lfAbs.read(); rrAbs.read(); lrAbs.read();

        // Motors (only for encoder)
        LionMotor rfMotor = LionMotor.withEncoder(hardwareMap, MotorConstants.Names.rightFront);
        LionMotor lfMotor = LionMotor.withEncoder(hardwareMap, MotorConstants.Names.leftFront);
        LionMotor rrMotor = LionMotor.withEncoder(hardwareMap, MotorConstants.Names.rightRear);
        LionMotor lrMotor = LionMotor.withEncoder(hardwareMap, MotorConstants.Names.leftRear);

        rfMotor.setReversed(MotorConstants.Reversed.rf);
        lfMotor.setReversed(MotorConstants.Reversed.lf);
        rrMotor.setReversed(MotorConstants.Reversed.rr);
        lrMotor.setReversed(MotorConstants.Reversed.lr);

        rfMotor.setReverseEncoder(true);
        rrMotor.setReverseEncoder(true);

        // Zero quadrature
        rfMotor.resetPositionTo(Zeroing.podAngle(rfAbs.position(), ConstantsStorage.get("rf",0.0)) * (4096.0 / 360.0));
        lfMotor.resetPositionTo(Zeroing.podAngle(lfAbs.position(), ConstantsStorage.get("lf",0.0)) * (4096.0 / 360.0));
        rrMotor.resetPositionTo(Zeroing.podAngle(rrAbs.position(), ConstantsStorage.get("rr",0.0)) * (4096.0 / 360.0));
        lrMotor.resetPositionTo(Zeroing.podAngle(lrAbs.position(), ConstantsStorage.get("lr",0.0)) * (4096.0 / 360.0));

        // Servos
        LionCRServo rfServo = new LionCRServo(hardwareMap, ServoConstants.Names.rightFront);
        LionCRServo lfServo = new LionCRServo(hardwareMap, ServoConstants.Names.leftFront);
        LionCRServo rrServo = new LionCRServo(hardwareMap, ServoConstants.Names.rightRear);
        LionCRServo lrServo = new LionCRServo(hardwareMap, ServoConstants.Names.leftRear);

        // PID controllers
        PID rfPID = new PID(0,0,0);
        PID lfPID = new PID(0,0,0);
        PID rrPID = new PID(0,0,0);
        PID lrPID = new PID(0,0,0);

        waitForStart();
        if (isStopRequested()) return;

        while (opModeIsActive()) {

            // Read absolute
            rfAbs.read(); lfAbs.read(); rrAbs.read(); lrAbs.read();

            // Current angles
            double rfAngle = wrapDeg(Zeroing.polarQuadrature(rfMotor.getPosition()));
            double lfAngle = wrapDeg(Zeroing.polarQuadrature(lfMotor.getPosition()));
            double rrAngle = wrapDeg(Zeroing.polarQuadrature(rrMotor.getPosition()));
            double lrAngle = wrapDeg(Zeroing.polarQuadrature(lrMotor.getPosition()));

            // Targets (closest equivalent)
            double rfTarget = closestEquivalent(Targets.rf, rfAngle);
            double lfTarget = closestEquivalent(Targets.lf, lfAngle);
            double rrTarget = closestEquivalent(Targets.rr, rrAngle);
            double lrTarget = closestEquivalent(Targets.lr, lrAngle);

            // === SET PID CONSTANTS EACH LOOP ===
            rfPID.setConstants(SwerveDrive.SwervePID.Pfr, 0, SwerveDrive.SwervePID.Dfr);
            lfPID.setConstants(SwerveDrive.SwervePID.Pfl, 0, SwerveDrive.SwervePID.Dfl);
            rrPID.setConstants(SwerveDrive.SwervePID.Prr, 0, SwerveDrive.SwervePID.Drr);
            lrPID.setConstants(SwerveDrive.SwervePID.Prl, 0, SwerveDrive.SwervePID.Drl);

            // PID outputs
            double rfOut = rfPID.calculate(rfAngle, rfTarget);
            double lfOut = lfPID.calculate(lfAngle, lfTarget);
            double rrOut = rrPID.calculate(rrAngle, rrTarget);
            double lrOut = lrPID.calculate(lrAngle, lrTarget);

            // kS feedforward (same as your code)
            rfOut += Math.signum(rfOut) * SwerveDrive.SwervePID.kSfr * TaskOpMode.Runtime.voltageCompensation;
            lfOut += Math.signum(lfOut) * SwerveDrive.SwervePID.kSfl * TaskOpMode.Runtime.voltageCompensation;
            rrOut += Math.signum(rrOut) * SwerveDrive.SwervePID.kSrr * TaskOpMode.Runtime.voltageCompensation;
            lrOut += Math.signum(lrOut) * SwerveDrive.SwervePID.kSrl * TaskOpMode.Runtime.voltageCompensation;

            // Deadband + limitband (copied logic)
            rfOut = applyLimits(rfOut, rfAngle, rfTarget);
            lfOut = applyLimits(lfOut, lfAngle, lfTarget);
            rrOut = applyLimits(rrOut, rrAngle, rrTarget);
            lrOut = applyLimits(lrOut, lrAngle, lrTarget);

            // Apply to servos
            rfServo.setPower(Double.isNaN(rfOut) ? 0 : rfOut);
            lfServo.setPower(Double.isNaN(lfOut) ? 0 : lfOut);
            rrServo.setPower(Double.isNaN(rrOut) ? 0 : rrOut);
            lrServo.setPower(Double.isNaN(lrOut) ? 0 : lrOut);

            // DO NOT DRIVE MOTORS
            rfMotor.setPower(0);
            lfMotor.setPower(0);
            rrMotor.setPower(0);
            lrMotor.setPower(0);

            // Telemetry
            telemetry.addLine("=== ABS ===");
            telemetry.addData("RF", rfAbs.position());
            telemetry.addData("LF", lfAbs.position());
            telemetry.addData("RR", rrAbs.position());
            telemetry.addData("LR", lrAbs.position());

            telemetry.addLine("=== QUAD (deg) ===");
            telemetry.addData("RF", rfAngle);
            telemetry.addData("LF", lfAngle);
            telemetry.addData("RR", rrAngle);
            telemetry.addData("LR", lrAngle);

            telemetry.addLine("=== ERROR ===");
            telemetry.addData("RF", wrapDeg(rfTarget - rfAngle));
            telemetry.addData("LF", wrapDeg(lfTarget - lfAngle));
            telemetry.addData("RR", wrapDeg(rrTarget - rrAngle));
            telemetry.addData("LR", wrapDeg(lrTarget - lrAngle));

            telemetry.update();
        }
    }

    private double applyLimits(double raw, double current, double target) {

        double error = Math.abs(wrapDeg(target - current));

        if (error < SwerveDrive.SwervePID.deadband) return 0;

        if (error < SwerveDrive.SwervePID.limitband) {
            if (raw < -SwerveDrive.SwervePID.limit) raw = -SwerveDrive.SwervePID.limit;
            if (raw >  SwerveDrive.SwervePID.limit) raw =  SwerveDrive.SwervePID.limit;
        }

        return raw;
    }
}
