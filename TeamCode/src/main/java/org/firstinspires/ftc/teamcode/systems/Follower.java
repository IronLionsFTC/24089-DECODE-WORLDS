package org.firstinspires.ftc.teamcode.systems;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.lioncore.math.types.Path;
import org.firstinspires.ftc.teamcode.lioncore.math.types.Position;
import org.firstinspires.ftc.teamcode.lioncore.math.types.Vector;
import org.firstinspires.ftc.teamcode.lioncore.systems.SystemBase;

public class Follower extends SystemBase {

    @Config
    public static class FollowerConstants {
        public static class TranslationPID {
            public static double P = 0.0;
            public static double I = 0.0;
            public static double D = 0.0;
        }
    }

    private SwerveDrive swerveDrive;
    private Path path;
    private double lastKValue;

    public Follower(SwerveDrive swerveDrive) {
        this.swerveDrive = swerveDrive;
    }

    @Override
    public void loadHardware(HardwareMap hardwareMap) {}

    @Override
    public void init() {}

    @Override
    public void update(Telemetry telemetry) {

        double targetKValue = Math.max(0, Math.min(1.0, lastKValue + 0.1));
        Position targetPosition = path.getTarget(targetKValue);
        Vector currentPosition = swerveDrive.getPosition().position;

        Vector driveVector = path.getTarget(1).position.sub(currentPosition);
        Vector translationalVector = targetPosition.position.sub(currentPosition);
        Vector finalVector = (driveVector.add(translationalVector)).normalised();

        double targetHeading = targetPosition.heading;

        swerveDrive.setTargetHeading(targetHeading);
    }
}
