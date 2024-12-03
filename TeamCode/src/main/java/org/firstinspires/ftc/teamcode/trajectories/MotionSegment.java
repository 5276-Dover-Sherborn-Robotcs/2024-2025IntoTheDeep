package org.firstinspires.ftc.teamcode.trajectories;

public class MotionSegment {

    double x0 = 0;
    double initial_heading = 0;
    double v0 = 0;
    double a = 0;
    double dt = 0;

    public MotionSegment(double v0, double a, double dt) {
        this(0, v0, a, dt);
    }

    public MotionSegment(double x0, double v0, double a, double dt) {
        this.x0 = x0;
        this.v0 = v0;
        this.a = a;
        this.dt = dt;
    }

    public boolean isDone(double time) {
        return time >= dt;
    }

    public double[] get(double time) {
        return new double[]{v0 * time + .5 * a * time * time, v0 + a * time, a};
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
