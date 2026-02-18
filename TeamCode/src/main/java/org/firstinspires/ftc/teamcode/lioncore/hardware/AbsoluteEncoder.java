package org.firstinspires.ftc.teamcode.lioncore.hardware;

import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class AbsoluteEncoder {
    private final AnalogInput analogSensor;
    private double position;

    public AbsoluteEncoder(HardwareMap hardwareMap, String name) {
        this.analogSensor = hardwareMap.get(AnalogInput.class, name);
    }

    public void read() {
        this.position = analogSensor.getVoltage();
    }

    public double position() {
        return this.position;
    }
}
