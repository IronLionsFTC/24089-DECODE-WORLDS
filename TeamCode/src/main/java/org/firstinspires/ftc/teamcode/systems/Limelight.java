package org.firstinspires.ftc.teamcode.systems;

import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.lioncore.systems.SystemBase;

public class Limelight extends SystemBase {

    private Limelight3A camera;

    @Override
    public void init() {
        setActivePipeline();
        this.camera.start();
    }

    @Override
    public void loadHardware(HardwareMap hardwareMap) {
        this.camera = hardwareMap.get(Limelight3A.class, "limelight");
    }

    @Override
    public void update(Telemetry telemetry, boolean useTelemetry) {
        if (useTelemetry) {
            telemetry.addData("Limelight Temp", this.temp());
            telemetry.addData("Limelight Pipeline", this.pipeline());
            telemetry.addData("Limelight Angle", this.angle());
        }
    }

    private void setActivePipeline() {
        camera.pipelineSwitch(5);
    }

    public Double angle() {
        return camera.getLatestResult().getTx();
    }

    public double pipeline() {
        return camera.getStatus().getPipelineIndex();
    }

    public double temp() {
        return camera.getStatus().getTemp();
    }
}
