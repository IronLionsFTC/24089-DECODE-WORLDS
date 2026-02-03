package org.firstinspires.ftc.teamcode.lioncore.hardware;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class LionCRServo {
    private CRServo hardware;
    private double power;

    public LionCRServo(HardwareMap hardwareMap, String name) {
        this.hardware = hardwareMap.get(CRServo.class, name);
    }

    public void setReversed(boolean reversed) {
        if (reversed) {
            this.hardware.setDirection(DcMotorSimple.Direction.REVERSE);
        } else {
            this.hardware.setDirection(DcMotorSimple.Direction.FORWARD);
        }
    }

    public void setPower(double newPower) {
        if (Math.abs(power - newPower) < 0.05 && newPower != 0) { return; }
        this.power = newPower;
        this.hardware.setPower(power);
    }

    public double getPower() {
        return this.power;
    }
}
