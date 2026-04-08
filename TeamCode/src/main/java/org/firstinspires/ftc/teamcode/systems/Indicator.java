package org.firstinspires.ftc.teamcode.systems;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.lioncore.hardware.LionServo;
import org.firstinspires.ftc.teamcode.lioncore.systems.SystemBase;

public class Indicator extends SystemBase {

    private LionServo light;

    public enum Colour {
        Green,
        Yellow,
        Orange,
        Red,
        Blue,
        Off,
        Purple
    }

    private Colour colour = Colour.Off;
    public Indicator() {}

    @Override
    public void loadHardware(HardwareMap hardwareMap) {
        this.light = LionServo.single(hardwareMap, "light", 0);
    }

    @Override
    public void init() {}

    @Override
    public void update(Telemetry telemetry, boolean useTelemetry) {
        double colour = 0;
        switch (this.colour) {
            case Off:
                colour = 0.0;
                break;
            case Red:
                colour = 0.277;
                break;
            case Green:
                colour = 0.500;
                break;
            case Yellow:
                colour = 0.388;
                break;
            case Blue:
                colour = 0.611;
                break;
            case Orange:
                colour = 0.333;
                break;
            case Purple:
                colour = 0.722;
                break;
        }
        this.light.setPosition(colour);
    }

    public void set(Colour colour) {
        this.colour = colour;
    }
}
