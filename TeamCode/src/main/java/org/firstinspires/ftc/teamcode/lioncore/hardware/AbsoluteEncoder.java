package org.firstinspires.ftc.teamcode.lioncore.hardware;

import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class AbsoluteEncoder {
    private AnalogInput analogSensor;
    private double position;

    public AbsoluteEncoder(HardwareMap hardwareMap, String name, double zeroPosition) {
        this.analogSensor = hardwareMap.get(AnalogInput.class, name);
    }

    public void read() {
        this.position = analogSensor.getVoltage();
    }

    public double position() {
        return this.position;
    }
}
