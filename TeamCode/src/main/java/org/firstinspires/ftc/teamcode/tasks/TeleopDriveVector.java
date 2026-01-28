package org.firstinspires.ftc.teamcode.tasks;

import org.firstinspires.ftc.teamcode.lioncore.math.types.Vector;
import org.firstinspires.ftc.teamcode.lioncore.tasks.Task;
import org.firstinspires.ftc.teamcode.systems.SwerveDrive;

import java.util.function.DoubleSupplier;

public class TeleopDriveVector extends Task {
    private SwerveDrive swerveDrive;
    private DoubleSupplier x;
    private DoubleSupplier y;

    public TeleopDriveVector(SwerveDrive swerveDrive, DoubleSupplier x, DoubleSupplier y) {
        this.swerveDrive = swerveDrive;
        this.x = x;
        this.y = y;
    }

    @Override
    public void run() {
        this.swerveDrive.setTargetVector(
                Vector.cartesian(this.x.getAsDouble(), this.y.getAsDouble())
        );
    }
}
