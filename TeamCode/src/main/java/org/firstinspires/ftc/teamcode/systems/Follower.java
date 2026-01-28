package org.firstinspires.ftc.teamcode.systems;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.lioncore.math.pid.PID;
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

    private PID drivePID;
    private PID translationalPID;

    public Follower(SwerveDrive swerveDrive) {
        this.swerveDrive = swerveDrive;
    }

    @Override
    public void loadHardware(HardwareMap hardwareMap) {}

    @Override
    public void init() {}

    @Override
    public void update(Telemetry telemetry) {

        Vector currentPosition = swerveDrive.getPosition().position;
        double kValue = path.getClosestK(currentPosition);

        Position targetPosition = path.getTarget(kValue);
        Vector driveVector = path.getTarget(kValue + 0.05).position.sub(targetPosition.position).normalised();
        Vector translationalVector = targetPosition.position.sub(currentPosition).normalised();

        double remainingDistance = path.distanceRemaining(kValue);
        double driveResponse = drivePID.calculate(remainingDistance, 0);
        double translationResponse = translationalPID.calculate(translationalVector.magnitude(), 0);

        Vector finalVector = (driveVector.multiply(driveResponse).add(translationalVector.multiply(translationResponse))).normalised();
        double targetHeading = targetPosition.heading;

        swerveDrive.setTargetHeading(targetHeading);
        swerveDrive.setTargetVector(finalVector);
    }
}
