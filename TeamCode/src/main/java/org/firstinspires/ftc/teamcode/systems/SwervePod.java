package org.firstinspires.ftc.teamcode.systems;

import org.firstinspires.ftc.teamcode.lioncore.hardware.LionMotor;
import org.firstinspires.ftc.teamcode.lioncore.hardware.LionServo;
import org.firstinspires.ftc.teamcode.lioncore.math.types.Vector2;

public class SwervePod {

    public static final double SERVO_RANGE_DEG = 270.0;
    public static final double GEAR_RATIO = 23.0 / 16.0 * 180 / 150;
    public static final double POD_RANGE_DEG  = SERVO_RANGE_DEG * GEAR_RATIO;
    public static final double POD_HALF_RANGE = POD_RANGE_DEG / 2.0;
    public static final boolean CCW_POSITIVE = false;

    // ── Hardware ──────────────────────────────────────────────────────────────

    private final LionMotor motor;
    private final LionServo servo;
    private final Vector2   offset;   // unit wheel position (±1, ±1) in robot frame

    public double currentAngle = 0.0;
    public double targetAngle  = 0.0;

    public final double offsetDeg;

    // ── Construction ──────────────────────────────────────────────────────────

    /**
     * @param motor  Drive wheel motor.
     * @param servo  Steering servo (position 0.5 = pod forward at init).
     * @param offset Normalised wheel position in robot frame, e.g. (±1, ±1).
     */
    public SwervePod(LionMotor motor, LionServo servo, Vector2 offset, double offsetDeg) {
        this.motor  = motor;
        this.servo  = servo;
        this.offset = offset;
        this.offsetDeg = offsetDeg;
        setServo(0.0);
    }

    public double update(Vector2 translation, double omega) {
        Vector2 rotComponent  = Vector2.cartesian( omega * offset.y(),
                -omega * offset.x());
        Vector2 wheelVelocity = translation.add(rotComponent);

        if (wheelVelocity.magnitude() < 1e-6) {
            return 0.0;
        }

        double desired = -wheelVelocity.polarDirection();
        double fwdTarget = closestEquivalent(desired,         currentAngle);
        double revTarget = closestEquivalent(desired + 180.0, currentAngle);

        double fwdTravel = Math.abs(wrapDeg(fwdTarget - currentAngle));
        double revTravel = Math.abs(wrapDeg(revTarget - currentAngle));

        double newAngle;
        double power;
        if (fwdTravel <= revTravel) {
            newAngle = fwdTarget;
            power    = +wheelVelocity.magnitude();
        } else {
            newAngle = revTarget;
            power    = -wheelVelocity.magnitude();
        }

        double delta      = Math.abs(wrapDeg(newAngle - currentAngle));
        double cosineFactor = Math.max(0.0, Math.cos(Math.toRadians(delta)));
        currentAngle = newAngle;
        targetAngle  = newAngle;
        setServo(newAngle);

        return power * cosineFactor;
    }

    public double updateIdle(boolean oPattern) {
        if (oPattern) {
            double tangent = wrapDeg(offset.polarDirection() + 90.0);
            double optA    = closestEquivalent(tangent,         currentAngle);
            double optB    = closestEquivalent(tangent + 180.0, currentAngle);
            double target  = Math.abs(wrapDeg(optA - currentAngle))
                    <= Math.abs(wrapDeg(optB - currentAngle)) ? optA : optB;
            currentAngle = target;
            targetAngle  = target;
            setServo(target);
        }
        return 0.0;
    }

    public void applyPower(double power) {
        motor.setPower(power);
    }

    private void setServo(double angleDeg) {
        angleDeg -= offsetDeg;
        double pos = 0.5 - angleDeg / POD_RANGE_DEG;

        while (pos > 1) {
            pos -= 360 / POD_RANGE_DEG;
        }

        while (pos < 0) {
            pos += 360 / POD_RANGE_DEG;
        }

        servo.setPosition(pos);
    }

    private static double wrapDeg(double a) {
        a = ((a % 360.0) + 360.0) % 360.0;
        if (a > 180.0) a -= 360.0;
        return a;
    }

    private static double closestEquivalent(double target, double reference) {
        return reference + wrapDeg(target - reference);
    }
}
