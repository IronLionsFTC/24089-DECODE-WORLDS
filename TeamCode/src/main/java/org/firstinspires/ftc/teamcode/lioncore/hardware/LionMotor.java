package org.firstinspires.ftc.teamcode.lioncore.hardware;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;

public class LionMotor {
    private List<DcMotorEx> motors;
    private double position;
    private double virtualOffset = 0;
    private double power;
    private boolean reverseEncoder = false;

    private LionMotor(HardwareMap hardwareMap, String... names) {
        this.motors = new ArrayList<>();
        for (String name : names) {
            this.motors.add(hardwareMap.get(DcMotorEx.class, name));
        }
        this.position = 0;
        this.power = 0;
    }

    private LionMotor(DcMotorEx... motors) {
        this.motors = new ArrayList<>();
        this.motors.addAll(Arrays.asList(motors));
        this.position = 0;
        this.power = 0;
    }

    public static LionMotor withoutEncoder(HardwareMap hardwareMap, String name) {
        return new LionMotor(hardwareMap, name);
    }

    public static LionMotor withEncoder(HardwareMap hardwareMap, String name) {
        return new LionMotor(hardwareMap, name);
    }

    public static LionMotor withEncoder(DcMotorEx hardware) {
        return new LionMotor(hardware);
    }

    public void setReverseEncoder(boolean reversed) {
        this.reverseEncoder = reversed;
    }

    /**
     * A LionMotor which operates multiple motors at once using the encoder of a singular motor.
     * @param hardwareMap The hardwaremap
     * @param names Any number of motors
     * @return
     */
    public static LionMotor masterSlaves(HardwareMap hardwareMap, String... names) {
        return new LionMotor(hardwareMap, names);
    }

    public void setPower(double power) {
        if (power != 0 && Math.abs(power - this.power) < 0.1) { return; }
        if (power == this.power) return;
        if (power >= 1 && this.power >= 1) return;
        if (power <=-1 && this.power <=-1) return;
        this.power = power;
        for (DcMotorEx motor : this.motors) {
            motor.setPower(power);
        }
    }

    public double getPower() {
        return this.power;
    }

    private double readEncoder() {
        if (this.motors.isEmpty()) return 0.0;
        this.position = this.motors.get(0).getCurrentPosition();
        if (reverseEncoder) return -position;
        else return position;
    }

    public double getPosition() {
        if (this.motors.isEmpty()) return 0.0;
        this.position = this.readEncoder() + this.virtualOffset;
        return this.cachedPosition();
    }

    /**
     * Calculate the current RPM of the motor using DcMotorEx sliding window getVelocity().
     * @param tpr Ticks per revolution of the required motor.
     * @return RPM (approx)
     */
    public double getVelocity(double tpr) {
        if (this.motors.isEmpty()) return 0.0;
        return this.motors.get(0).getVelocity() * 60 * (1 / tpr);
    }

    /**
     * Calculate the average velocity of all motors on the system
     * @return RPM
     */
    public double getVelocityAverage(double tpr) {
        if (this.motors.isEmpty()) return 0.0;
        double sum = 0;
        for (DcMotorEx motor : this.motors) {
            sum += motor.getVelocity() * 60 * (1 / tpr);
        }
        return sum / (double)this.motors.size();
    }

    public double cachedPosition() {
        return this.position;
    }

    public void resetPosition() {
        if (this.motors.isEmpty()) return;
        this.motors.get(0).setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        this.motors.get(0).setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
    }

    public void resetPositionTo(double currentPosition) {
        this.virtualOffset = 0;
        this.virtualOffset = currentPosition - this.getPosition();
    }

    public void setReversed(boolean... reversed) {
        assert reversed.length == motors.size();
        for (int idx = 0; idx < this.motors.size(); idx++) {
            if (reversed[idx]) this.motors.get(idx).setDirection(DcMotorSimple.Direction.REVERSE);
            else this.motors.get(idx).setDirection(DcMotorSimple.Direction.FORWARD);
        }
    }

    public void setZPB(DcMotorEx.ZeroPowerBehavior zpb) {
        for (DcMotorEx motor : this.motors) {
            motor.setZeroPowerBehavior(zpb);
        }
    }

    public double getAmps() {
        if (this.motors.isEmpty()) return 0;
        double sum = 0;
        for (DcMotorEx motor : this.motors) {
            sum += motor.getCurrent(CurrentUnit.AMPS);
        }
        return sum / (this.motors.size());
    }

    public Encoder yieldEncoder(int n) {
        return new Encoder(LionMotor.withEncoder(this.motors.get(n)));
    }
}
