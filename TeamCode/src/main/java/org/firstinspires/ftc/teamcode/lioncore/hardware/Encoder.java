package org.firstinspires.ftc.teamcode.lioncore.hardware;

public class Encoder {
    private final LionMotor source;

    public Encoder(LionMotor source) {
        this.source = source;
    }

    public double getPosition() {
        return this.source.getPosition();
    }

    public void setPosition(double position) {
        this.source.resetPositionTo(position);
    }
}
