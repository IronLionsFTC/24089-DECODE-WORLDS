package org.firstinspires.ftc.teamcode.Opmodes.Debug;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.lioncore.hardware.AbsoluteEncoder;
import org.firstinspires.ftc.teamcode.lioncore.hardware.Encoder;
import org.firstinspires.ftc.teamcode.lioncore.hardware.LionCRServo;
import org.firstinspires.ftc.teamcode.lioncore.hardware.LionMotor;
import org.firstinspires.ftc.teamcode.lioncore.math.pid.PID;
import org.firstinspires.ftc.teamcode.lioncore.system.ConstantsStorage;
import org.firstinspires.ftc.teamcode.parameters.MotorConstants;
import org.firstinspires.ftc.teamcode.parameters.ServoConstants;

@TeleOp
public class ZeroTurret extends OpMode {

    private AbsoluteEncoder absolute;
    private Encoder quadrature;

    private LionCRServo leftTurretServo;
    private LionCRServo rightTurretServo;

    @Config
    public static class TurretPID {
        public static double P = 0.0075;
        public static double I = 0.0;
        public static double D = 0.00015;
        public static double kS = 0.05;
        public static double latency = 0;
    }

    @Override
    public void init() {
        this.absolute = new AbsoluteEncoder(hardwareMap, "turretAbsolute");
        LionMotor rightShooterMotor = LionMotor.withEncoder(hardwareMap, MotorConstants.Names.rightShooterMotor);
        rightShooterMotor.setReversed(MotorConstants.Reversed.rightShooterMotor);
        rightShooterMotor.setReverseEncoder(true);
        this.quadrature = new Encoder(rightShooterMotor);

        absolute.read();

        ConstantsStorage.save("turretZeroVoltage", absolute.position());
        double absolutePosition = ((absolute.position() - ServoConstants.Zero.turret) / 3.3) * 360;

        while (absolutePosition < -180.0) absolutePosition += 360;
        while (absolutePosition >  180.0) absolutePosition -= 360;

        double inTicks = absolutePosition / 360 * 4096;
        quadrature.setPosition(inTicks);

        this.leftTurretServo = new LionCRServo(hardwareMap, ServoConstants.Names.leftTurretServo);
        this.rightTurretServo = new LionCRServo(hardwareMap, ServoConstants.Names.rightTurretServo);

    }

    @Override
    public void loop() {
        absolute.read();
        telemetry.addData("Voltage", absolute.position());

        double absolutePosition = ((absolute.position() - ServoConstants.Zero.turret) / 3.3) * 360;
        double quadraturePosition = quadrature.getPosition() / 4096 * 360;

        while (absolutePosition < -180.0) absolutePosition += 360;
        while (absolutePosition >  180.0) absolutePosition -= 360;
        while (quadraturePosition < -180.0) quadraturePosition += 360;
        while (quadraturePosition >  180.0) quadraturePosition -= 360;

        telemetry.addData("Absolute", absolutePosition);
        telemetry.addData("Quadrature", quadraturePosition);

        telemetry.update();

        this.leftTurretServo.setPower(0);
        this.rightTurretServo.setPower(0);
    }
}
