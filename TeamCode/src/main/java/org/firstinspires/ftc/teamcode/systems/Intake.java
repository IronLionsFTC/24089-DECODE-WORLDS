package org.firstinspires.ftc.teamcode.systems;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.lioncore.hardware.LionMotor;
import org.firstinspires.ftc.teamcode.lioncore.systems.SystemBase;
import org.firstinspires.ftc.teamcode.parameters.MotorConstants;

public class Intake extends SystemBase {

    public enum State {
        IntakingEmpty,
        IntakeOnly,
        Off,
        Shooting
    }

    private LionMotor intakeMotor;
    private LionMotor transferMotor;
    private State state;

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
    }

    @Override
    public void init() {

    }

    @Override
    public void update(Telemetry telemetry) {

    }
}
