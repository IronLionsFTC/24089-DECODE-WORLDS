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

    public enum Team {
        Red,
        Blue
    }

    private Limelight3A camera;
    private final Team team;

    private final double s;
    private final double c;

    public Limelight(Team team) {
        this.position = new Position(0, 0, 0);
        this.cache = new Position(0, 0, 0);
        this.team = team;

        if (team == Team.Blue) {
            this.s = Math.sin(Math.toRadians(35));
            this.c = Math.cos(Math.toRadians(35));
        } else {
            this.s = Math.sin(Math.toRadians(55));
            this.c = Math.cos(Math.toRadians(55));
        }
    }

    @Config
    public static class LimelightOffset {
        public static double x = -300;
        public static double y = 300;
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
        // Set initial pipeline but DO NOT start vision yet
        setActivePipeline();
    }

    private void setActivePipeline() {
        if (team == Team.Blue) camera.pipelineSwitch(0);
        else camera.pipelineSwitch(1);
    }

    private void setIdlePipeline() {
        camera.pipelineSwitch(7); // empty pipeline = reduced load
    }

    @Override
    public void update(Telemetry telemetry, boolean updateTelemetry) {

        if (!running) return; // HARD STOP: no I/O at all when disabled

        LLResult result = camera.getLatestResult();
        if (result == null || !result.isValid()) return;

        List<LLResultTypes.FiducialResult> fiducials = result.getFiducialResults();
        if (fiducials.isEmpty()) return;

        // Only process first fiducial for determinism + speed
        LLResultTypes.FiducialResult fiducial = fiducials.get(0);
        Pose3D pose = fiducial.getRobotPoseTargetSpace();

        double y = pose.getPosition().z * -1000;
        double x = pose.getPosition().x * -1000;

        double xp = y * s + x * s;
        double yp = x * c - y * c;

        if (team == Team.Red) {
            double temp = xp;
            xp = yp;
            yp = temp;
        }

        cache.update(
                -yp + LimelightOffset.y,
                -xp + LimelightOffset.x,
                180 - (pose.getOrientation().getPitch(AngleUnit.DEGREES) + 35)
        );

        // FIXED: proper validity logic (no self-shadowing bug anymore)
        isValid =
                cache.position.x() != 0 &&
                        cache.position.y() != 0 &&
                        cache.heading != 0;

        if (isValid) {
            position.update(
                    cache.position.x(),
                    cache.position.y(),
                    cache.heading
            );
        }
    }

    public void start() {
        cache.update(0, 0, 0);
        position.update(0, 0, 0);

        running = true;
        isValid = false;

        setActivePipeline();
    }

    public void stop() {
        running = false;
        isValid = false;

        setIdlePipeline(); // IMPORTANT: reduces internal Limelight processing load
    }
}
