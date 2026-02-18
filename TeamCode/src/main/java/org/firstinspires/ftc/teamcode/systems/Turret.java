package org.firstinspires.ftc.teamcode.systems;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.lioncore.hardware.AbsoluteEncoder;
import org.firstinspires.ftc.teamcode.lioncore.hardware.LionCRServo;
import org.firstinspires.ftc.teamcode.lioncore.systems.SystemBase;
import org.firstinspires.ftc.teamcode.parameters.ServoConstants;

public class Turret extends SystemBase {

    private LionCRServo leftTurretServo;
    private LionCRServo rightTurretServo;

    public Turret() {}

    @Override
    public void loadHardware(HardwareMap hardwareMap) {
        this.leftTurretServo = new LionCRServo(hardwareMap, ServoConstants.Names.leftTurretServo);
        this.rightTurretServo = new LionCRServo(hardwareMap, ServoConstants.Names.rightTurretServo);

        AbsoluteEncoder readZero = new AbsoluteEncoder(hardwareMap, "turretAbsolute");
    }

    @Override
    public void init() {

    }

    @Override
    public void update(Telemetry telemetry) {

    }

}
