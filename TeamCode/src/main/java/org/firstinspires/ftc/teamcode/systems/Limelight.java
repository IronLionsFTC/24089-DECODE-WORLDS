package org.firstinspires.ftc.teamcode.systems;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.teamcode.lioncore.math.types.Position;
import org.firstinspires.ftc.teamcode.lioncore.systems.SystemBase;

import java.util.List;

public class Limelight extends SystemBase {

    private Limelight3A camera;
    public Limelight() {
        this.position = new Position(0, 0,0);
        this.cache = new Position(0, 0,0);
    }

    @Config
    public static class LimelightOffset {
        public static double x = -300;
        public static double y =  300;
    }

    public final Position cache;
    public final Position position;
    public boolean isValid = false;
    public boolean running = false;

    @Override
    public void loadHardware(HardwareMap hardwareMap) {
        this.camera = hardwareMap.get(Limelight3A.class, "limelight");
    }

    @Override
    public void init() {
        this.camera.start();
        this.camera.pipelineSwitch(7);
        this.camera.pause();
    }

    @Override
    public void update(Telemetry telemetry, boolean updateTelemetry) {

        if (running && !isValid) {

            LLResult result = this.camera.getLatestResult();
            if (!result.isValid()) return;
            List<LLResultTypes.FiducialResult> fedres = result.getFiducialResults();
            for (LLResultTypes.FiducialResult feducial : fedres) {
                Pose3D pose = feducial.getRobotPoseTargetSpace();

                double y = pose.getPosition().z * -1000;
                double x = pose.getPosition().x * -1000;

                double s = Math.sin(Math.toRadians(35));
                double c = Math.cos(Math.toRadians(35));

                double xp = y * s + x * s;
                double yp = x * c - y * c;

                this.cache.update(
                        -yp + LimelightOffset.y,
                        -xp + LimelightOffset.x,
                        180 - (pose.getOrientation().getPitch(AngleUnit.DEGREES) + 35));

                this.isValid = cache.position.x() != 0 && cache.position.y() != 0 && cache.heading != 0;
                if (this.isValid) {
                    this.position.update(
                            this.cache.position.x(),
                            this.cache.position.y(),
                            this.cache.heading
                    );
                }
            }
        }
    }

    // TOOD - Red side
    public void start() {
        this.running = true;
        this.isValid = false;
        this.camera.start();
        this.camera.pipelineSwitch(0);
    }

    public void stop() {
        this.running = false;
        this.isValid = false;
        this.camera.pipelineSwitch(7);
        this.camera.pause();
    }
}
