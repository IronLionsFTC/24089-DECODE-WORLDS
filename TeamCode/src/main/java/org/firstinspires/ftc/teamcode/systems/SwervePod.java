package org.firstinspires.ftc.teamcode.systems;

import org.firstinspires.ftc.teamcode.lioncore.hardware.LionMotor;
import org.firstinspires.ftc.teamcode.lioncore.hardware.LionServo;
import org.firstinspires.ftc.teamcode.lioncore.math.types.Vector2;

public class SwervePod {

    public static final double SERVO_RANGE_DEG = 270.0;
    public static final double GEAR_RATIO = 23.0 / 16.0 * 180 / 150;
    public static final double POD_RANGE_DEG  = SERVO_RANGE_DEG * GEAR_RATIO;
    public static final double POD_HALF_RANGE = POD_RANGE_DEG / 2.0;

    private final LionMotor motor;
    private final LionServo servo;
    private final Vector2   offset;

    public double currentAngle = 0.0;
    public double targetAngle  = 0.0;

    public final double offsetDeg;

    public SwervePod(LionMotor motor, LionServo servo, Vector2 offset, double offsetDeg) {
        this.motor     = motor;
        this.servo     = servo;
        this.offset    = offset;
        this.offsetDeg = offsetDeg;
        setServo(0.0);
    }

    public double update(Vector2 translation, double omega) {
        Vector2 rotComponent  = Vector2.cartesian(omega * offset.y(), -omega * offset.x());
        Vector2 wheelVelocity = translation.add(rotComponent);

        if (wheelVelocity.magnitude() < 1e-6) {
            return 0.0;
        }

        double desired = -wheelVelocity.polarDirection();

        double bestTarget   = currentAngle;
        double bestTravel   = Double.MAX_VALUE;
        double bestMotorDir = 1.0;

        for (int i = 0; i < 2; i++) {
            double heading  = desired + i * 180.0;
            double motorDir = (i == 0) ? +1.0 : -1.0;
            double base     = currentAngle + wrapDeg(heading - currentAngle);

            for (int k = -1; k <= 1; k++) {
                double candidate = base + k * 360.0;
                if (candidate >= -POD_HALF_RANGE && candidate <= POD_HALF_RANGE) {
                    double travel = Math.abs(candidate - currentAngle);
                    if (travel < bestTravel) {
                        bestTravel   = travel;
                        bestTarget   = candidate;
                        bestMotorDir = motorDir;
                    }
                }
            }
        }

        double cosineFactor = Math.max(0.0, Math.cos(Math.toRadians(bestTravel)));
        currentAngle = bestTarget;
        targetAngle  = bestTarget;
        setServo(bestTarget);

        return bestMotorDir * wheelVelocity.magnitude() * cosineFactor;
    }

    public double updateIdle(boolean oPattern) {
        if (oPattern) {
            double tangent    = wrapDeg(offset.polarDirection() + 90.0);
            double bestTarget = currentAngle;
            double bestTravel = Double.MAX_VALUE;

            for (int i = 0; i < 2; i++) {
                double heading = tangent + i * 180.0;
                double base    = currentAngle + wrapDeg(heading - currentAngle);

                for (int k = -1; k <= 1; k++) {
                    double candidate = base + k * 360.0;
                    if (candidate >= -POD_HALF_RANGE && candidate <= POD_HALF_RANGE) {
                        double travel = Math.abs(candidate - currentAngle);
                        if (travel < bestTravel) {
                            bestTravel = travel;
                            bestTarget = candidate;
                        }
                    }
                }
            }

            currentAngle = bestTarget;
            targetAngle  = bestTarget;
            setServo(bestTarget);
        }
        return 0.0;
    }

    public double updatePreload(Vector2 preload) {
        setServo(preload.polarDirection());
        return 0.0;
    }

    public void applyPower(double power) {
        motor.setPower(power);
    }

    private void setServo(double angleDeg) {
        angleDeg -= offsetDeg;
        double pos = 0.5 - angleDeg / POD_RANGE_DEG;

        while (pos > 1) {
            pos -= 360.0 / POD_RANGE_DEG;
        }

        while (pos < 0) {
            pos += 360.0 / POD_RANGE_DEG;
        }

        servo.setPosition(pos);
    }

    private static double wrapDeg(double a) {
        a = ((a % 360.0) + 360.0) % 360.0;
        if (a > 180.0) a -= 360.0;
        return a;
    }
}
