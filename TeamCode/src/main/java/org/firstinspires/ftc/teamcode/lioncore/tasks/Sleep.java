package org.firstinspires.ftc.teamcode.lioncore.tasks;

public class Sleep extends Task {

    private final double seconds;
    private long endTime;

    public Sleep(double seconds) {
        this.seconds = seconds;
    }

    public void init() {
        long startTime = System.nanoTime();
        this.endTime = startTime + (long)(this.seconds * 1_000_000_000);
    }

    public boolean finished() {
        return System.nanoTime() >= this.endTime;
    }
}
