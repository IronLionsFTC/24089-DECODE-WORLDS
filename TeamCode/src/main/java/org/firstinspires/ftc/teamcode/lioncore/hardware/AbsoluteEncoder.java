package org.firstinspires.ftc.teamcode.lioncore.hardware;

import com.qualcomm.robotcore.hardware.AnalogSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class AbsoluteEncoder {
    private AnalogSensor analogSensor;
    private double position;

    public AbsoluteEncoder(HardwareMap hardwareMap, String name, double zeroPosition) {
        this.analogSensor = hardwareMap.get(AnalogSensor.class, name);
    }

    public void read() {
        this.position = analogSensor.readRawVoltage();
    }

    public double position() {
        return this.position;
    }
}
