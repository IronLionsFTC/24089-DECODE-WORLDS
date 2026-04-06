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
    }

    @Config
    public static class LimelightOffset {
        public static double x = -300;
        public static double y = -300;
    }

    public final Position position;
    public boolean isValid = false;

    @Override
    public void loadHardware(HardwareMap hardwareMap) {
        this.camera = hardwareMap.get(Limelight3A.class, "limelight");
    }

    @Override
    public void init() {
        this.camera.start();
        this.camera.pipelineSwitch(0);
    }

    @Override
    public void update(Telemetry telemetry, boolean updateTelemetry) {
        LLResult result = this.camera.getLatestResult();
        if (!result.isValid()) return;
        List<LLResultTypes.FiducialResult> fedres = result.getFiducialResults();
        for (LLResultTypes.FiducialResult feducial : fedres) {
            Pose3D pose = feducial.getRobotPoseTargetSpace();

            double y = pose.getPosition().z * - 1000;
            double x = pose.getPosition().x * - 1000;

            double s = Math.sin(Math.toRadians(35));
            double c = Math.cos(Math.toRadians(35));

            double xp = y * s + x * s;
            double yp = x * c - y * c;

            this.position.update(
                    -yp + x,
                    -xp + y,
            180 - (pose.getOrientation().getPitch(AngleUnit.DEGREES) + 35));

            telemetry.addData("TX", position.position.x());
            telemetry.addData("TY", position.position.y());
            telemetry.addData("T_YAW", pose.getOrientation().getPitch(AngleUnit.DEGREES));

            if (position.position.x() != 0 && position.position.y() != 0 && position.heading != 0) {
                this.isValid = true;
            } else {
                this.isValid = false;
            }
        }
    }
}
