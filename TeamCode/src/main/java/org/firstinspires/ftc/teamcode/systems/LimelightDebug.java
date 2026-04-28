package org.firstinspires.ftc.teamcode.systems;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.lioncore.systems.SystemBase;

public class LimelightDebug extends SystemBase {
    private Limelight limelight;
    public LimelightDebug() {}

    @Override
    public void loadHardware(HardwareMap hardwareMap) {
        this.limelight = new Limelight(hardwareMap);
    }

    @Override
    public void init() {}

    @Override
    public void update(Telemetry telemetry, boolean useTelemetry) {
        if (useTelemetry) {
            telemetry.addData("Limelight", limelight.angle());
            telemetry.addData("LimelightPipeline", limelight.pipeline());
            telemetry.addData("Temp", limelight.temp());
        }
    }
}
