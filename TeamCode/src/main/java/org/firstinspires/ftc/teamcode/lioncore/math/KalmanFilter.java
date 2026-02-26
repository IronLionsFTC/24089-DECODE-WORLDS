package org.firstinspires.ftc.teamcode.lioncore.math;

public class KalmanFilter {
    private double q; // process noise - how much you expect velocity to change per loop
    private double r; // measurement noise - variance of your RPM measurement
    private double p; // estimate uncertainty
    private double x; // current estimate

    public KalmanFilter(double q, double r, double initialEstimate) {
        this.q = q;
        this.r = r;
        this.p = r;
        this.x = initialEstimate;
    }

    public double update(double measurement) {
        // Predict
        p = p + q;

        // Update
        double k = p / (p + r);  // Kalman gain
        x = x + k * (measurement - x);
        p = (1 - k) * p;

        return x;
    }

    public void setQ(double q) { this.q = q; }
    public void setR(double r) { this.r = r; }
}
