package org.firstinspires.ftc.teamcode.Opmodes.Debug;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.lioncore.hardware.AbsoluteEncoder;
import org.firstinspires.ftc.teamcode.lioncore.hardware.Encoder;
import org.firstinspires.ftc.teamcode.lioncore.hardware.LionMotor;
import org.firstinspires.ftc.teamcode.parameters.MotorConstants;
import org.firstinspires.ftc.teamcode.parameters.ServoConstants;

@TeleOp
public class ZeroTurret extends OpMode {

    private AbsoluteEncoder absolute;
    private LionMotor rightShooterMotor;
    private Encoder quadrature;

    @Override
    public void init() {
        this.absolute = new AbsoluteEncoder(hardwareMap, "turretAbsolute");
        this.rightShooterMotor = LionMotor.withEncoder(hardwareMap, MotorConstants.Names.rightShooterMotor);
        this.rightShooterMotor.setReversed(MotorConstants.Reversed.rightShooterMotor);
        this.rightShooterMotor.setReverseEncoder(true);
        this.quadrature = new Encoder(rightShooterMotor);

        absolute.read();
        double absolutePosition = ((absolute.position() - ServoConstants.Zero.turret) / 3.3) * 360;

        while (absolutePosition < -180.0) absolutePosition += 360;
        while (absolutePosition >  180.0) absolutePosition -= 360;

        double inTicks = absolutePosition / 360 * 4096;
        quadrature.setPosition(inTicks);
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

    }
}
