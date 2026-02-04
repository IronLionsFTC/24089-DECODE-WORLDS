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
    public static class TranslationPID {
        public static double P = 0.0;
        public static double I = 0.0;
        public static double D = 0.0;
    }

    @Config
    public static class DrivePID {
        public static double P = 0.0;
        public static double I = 0.0;
        public static double D = 0.0;
    }

    private final SwerveDrive swerveDrive;

    private Path path;
    private final PID drivePID;
    private final PID translationalPID;

    private final Position targetPosition;
    private final Vector driveVector;
    private final Vector translationalVector;
    private final Vector finalVector;
    private final Position temp;

    public Follower(SwerveDrive swerveDrive) {
        this.swerveDrive = swerveDrive;
        this.targetPosition = new Position(0, 0, 0);
        this.driveVector = Vector.cartesian(0, 0);
        this.translationalVector = Vector.cartesian(0, 0);
        this.temp = new Position(0, 0, 0);
        this.finalVector = Vector.cartesian(0, 0);

        this.drivePID = new PID(0, 0, 0);
        this.translationalPID = new PID(0, 0, 0);
    }

    @Override
    public void loadHardware(HardwareMap hardwareMap) {}

    @Override
    public void init() {}

    @Override
    public void update(Telemetry telemetry) {
        if (path == null) return;

        this.drivePID.setConstants(
                DrivePID.P,
                DrivePID.I,
                DrivePID.D
        );

        this.translationalPID.setConstants(
                TranslationPID.P,
                TranslationPID.I,
                TranslationPID.D
        );

        // Calculate the parametric variables and discrete approximation of tangent to the path.
        double kValue = path.getClosestK();
        path.getTarget(kValue, targetPosition);
        path.getTarget(kValue + 0.05, temp);

        // Distance to the end of the path
        double distance = path.distanceRemaining();

        // Calculate the raw drive and translational vectors. If the drive vector is close to the end, drive there directly.
        if (distance > 200) temp.position.sub_into(targetPosition.position, driveVector);
        else path.set_to_end(driveVector);
        targetPosition.position.sub_into(SwerveDrive.PinpointCache.position.position, translationalVector);
        driveVector.normalise();
        translationalVector.normalise();

        // Minimise distance remaining and translational error and scale normalised vectors
        double driveResponse = drivePID.calculate(path.distanceRemaining(), 0);
        double translationResponse = translationalPID.calculate(translationalVector.magnitude(), 0);
        driveVector.multiply_mut(driveResponse);
        translationalVector.multiply_mut(translationResponse);
        driveVector.add_into(translationalVector, finalVector);

        // Calculate the target heading
        double targetHeading = targetPosition.heading;

        // Translate the field centric movement back into robot centric inputs
        double headingRadians = Math.toRadians(SwerveDrive.PinpointCache.position.heading);
        double c = Math.cos(headingRadians);
        double s = Math.sin(headingRadians);

        temp.update(
                finalVector.x() * c + finalVector.y() * s,
                -finalVector.x() * s + finalVector.y() * c,
                0
        );

        swerveDrive.setTargetHeading(targetHeading);
        swerveDrive.setTargetVector(temp.position);
    }

    public double getDistance() {
        return this.path.distanceRemaining();
    }

    public void stop() {
        this.path = null;
    }

    public void follow(Path path) {
        this.path = path;
    }
}
