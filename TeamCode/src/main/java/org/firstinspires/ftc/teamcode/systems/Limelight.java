package org.firstinspires.ftc.teamcode.systems;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Limelight {

    private Limelight3A camera;

    public Limelight(HardwareMap hardwareMap) {
        this.camera = hardwareMap.get(Limelight3A.class, "limelight");
        setActivePipeline();
    }

    private void setActivePipeline() {
        camera.pipelineSwitch(5);
    }

    public Double angle(boolean updateTelemetry) {
        LLResult result = camera.getLatestResult();
        if (result == null || !result.isValid()) return null;
        return result.getTx();
    }
}
