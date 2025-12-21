package org.firstinspires.ftc.teamcode.systems;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.lioncore.hardware.LionMotor;
import org.firstinspires.ftc.teamcode.lioncore.hardware.LionServo;
import org.firstinspires.ftc.teamcode.lioncore.systems.SystemBase;

public class Shooter extends SystemBase {
    private LionMotor motors;
    private LionServo hoodServo;

    public Shooter shooter() {}

    @Override
    public void loadHardware(HardwareMap hardwareMap) {
        this.motors = LionMotor.masterSlaves(hardwareMap, )
    }
}
