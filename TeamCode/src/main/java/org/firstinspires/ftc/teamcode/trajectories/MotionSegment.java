package org.firstinspires.ftc.teamcode.trajectories;

public class MotionSegment {

    double x0 = 0;
    double x1 = 0;
    double v0 = 0;
    double v1 = 0;
    double a = 0;
    double t0 = 0;
    double dt = 0;

    public MotionSegment(double v0, double a, double dt) {
        this(v0, a, dt, 0, 0);
    }

    public MotionSegment(double v0, double a, double dt, double t0) {
        this(v0, a, dt, t0, 0);
    }

    public MotionSegment(double v0, double a, double dt, double t0, double x0) {
        this.x0 = x0;
        this.v0 = v0;
        this.a = a;
        this.t0 = t0;
        this.dt = dt;
        x1 = v0 * dt + .5 * a * dt * dt;
        v1 = v0 + a * dt;
    }

    public boolean isDone(double time) {
        return time >= t0 + dt;
    }

    public double[] get(double time) {
        return new double[]{v0 * time + .5 * a * time * time, v0 + a * time};
    }

    public double get_pos(double time) {
        return v0 * time + .5 * a * time * time;
    }

    public double get_vel(double time) {
        return v0 + a * time;
    }
    public double get_accel(double time) {
        return a;
    }
}
