package org.firstinspires.ftc.teamcode.systems;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.lioncore.math.types.Path;
import org.firstinspires.ftc.teamcode.lioncore.math.types.Position;
import org.firstinspires.ftc.teamcode.lioncore.math.types.Vector;
import org.firstinspires.ftc.teamcode.lioncore.systems.SystemBase;

public class Follower extends SystemBase {

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

        Vector driveVector = targetPosition.position.sub(swerveDrive.getPosition().position);
        double targetHeading = targetPosition.heading;

        swerveDrive.setTargetHeading(targetHeading);
    }
}
