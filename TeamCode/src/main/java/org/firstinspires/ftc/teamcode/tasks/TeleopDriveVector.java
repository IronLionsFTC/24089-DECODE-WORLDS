package org.firstinspires.ftc.teamcode.tasks;

import org.firstinspires.ftc.teamcode.lioncore.math.types.Vector2;
import org.firstinspires.ftc.teamcode.lioncore.tasks.Task;
import org.firstinspires.ftc.teamcode.systems.SwerveDrive;

import java.util.function.DoubleSupplier;

public class TeleopDriveVector extends Task {
    private final Vector2 driveVector;
    private final SwerveDrive swerveDrive;
    private final DoubleSupplier x;
    private final DoubleSupplier y;

    public TeleopDriveVector(SwerveDrive swerveDrive, DoubleSupplier x, DoubleSupplier y) {
        this.swerveDrive = swerveDrive;
        this.x = x;
        this.y = y;
        this.driveVector = Vector2.cartesian(0, 0);
    }

    @Override
    public void run() {
        driveVector.update(
                x.getAsDouble(),
                y.getAsDouble()
        );
        this.swerveDrive.setTargetVector(driveVector);
    }
}
