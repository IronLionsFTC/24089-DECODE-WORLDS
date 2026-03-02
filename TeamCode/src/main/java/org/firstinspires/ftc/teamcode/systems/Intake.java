package org.firstinspires.ftc.teamcode.systems;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.lioncore.hardware.LionServo;
import org.firstinspires.ftc.teamcode.lioncore.hardware.LionMotor;
import org.firstinspires.ftc.teamcode.lioncore.math.types.Vector2;
import org.firstinspires.ftc.teamcode.lioncore.systems.SystemBase;
import org.firstinspires.ftc.teamcode.parameters.MotorConstants;
import org.firstinspires.ftc.teamcode.parameters.ServoConstants;
import org.firstinspires.ftc.teamcode.projectileMotion.ProjectileMotion;

public class Intake extends SystemBase {

    public enum State {
        IntakingEmpty,
        IntakeOnly,
        Off,
        Shooting,
        TopOnly,
        Reverse
    }

    private LionMotor intakeMotor;
    private LionMotor transferMotor;
    private LionServo blocker;
    private State state;

    @Config
    public static class IntakeConstants {
        public static double currentThreshold = 6;
        public static double intakeThreshold = 5;
    }

    public Intake() {
        this.state = State.Off;
    }

    @Override
    public void loadHardware(HardwareMap hardwareMap) {
        this.intakeMotor = LionMotor.withoutEncoder(hardwareMap, MotorConstants.Names.intakeMotor);
        this.transferMotor = LionMotor.withoutEncoder(hardwareMap, MotorConstants.Names.transferMotor);
        this.intakeMotor.setReversed(MotorConstants.Reversed.intakeMotor);
        this.transferMotor.setReversed(MotorConstants.Reversed.transferMotor);
        this.intakeMotor.setZPB(MotorConstants.ZPB.intakeMotors);
        this.transferMotor.setZPB(MotorConstants.ZPB.intakeMotors);
        this.blocker = LionServo.single(hardwareMap, ServoConstants.Names.blocker, ServoConstants.Positions.blockerOpen);
    }

    @Override
    public void init() {

    }

    @Override
    public void update(Telemetry telemetry, boolean useTelemetry) {

        double current;

        switch (this.state) {
            case Off:
                this.intakeMotor.setPower(0);
                this.transferMotor.setPower(0);
                this.blocker.setPosition(ServoConstants.Positions.blockerClosed);
                break;

            case Shooting:
                double power = 1 - ProjectileMotion.ShootOnTheMoveConstants.lastDistance / 12000;
                this.intakeMotor.setPower(power);
                this.transferMotor.setPower(power);
                this.blocker.setPosition(ServoConstants.Positions.blockerOpen);
                break;

            case IntakeOnly:
                this.intakeMotor.setPower(1);
                this.transferMotor.setPower(0);
                this.blocker.setPosition(ServoConstants.Positions.blockerClosed);

                current = intakeMotor.getAmps();
                if (current > IntakeConstants.intakeThreshold) this.state = State.Off;

                break;

            case IntakingEmpty:
                this.intakeMotor.setPower(1);
                this.transferMotor.setPower(1);
                this.blocker.setPosition(ServoConstants.Positions.blockerClosed);

                current = transferMotor.getAmps();
                if (current > IntakeConstants.currentThreshold) this.state = State.IntakeOnly;

                break;

            case TopOnly:
                this.intakeMotor.setPower(0);
                this.transferMotor.setPower(1);
                this.blocker.setPosition(ServoConstants.Positions.blockerOpen);
                break;

            case Reverse:
                this.intakeMotor.setPower(0.5);
                this.transferMotor.setPower(-1);
                this.blocker.setPosition(ServoConstants.Positions.blockerOpen);
                break;
        }
    }

    public void setState(State state) {
        this.state = state;
    }

    public State getState() {
        return this.state;
    }
}
