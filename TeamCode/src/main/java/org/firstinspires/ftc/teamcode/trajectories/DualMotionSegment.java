package org.firstinspires.ftc.teamcode.trajectories;

public class DualMotionSegment {

    public double x0 = 0;
    public double r0 = 0;
    public double v_t = 0;
    public double v_r = 0;
    public double a_t = 0;
    public double a_r = 0;
    public double theta = 0;
    public double dt = 0;

    public DualMotionSegment(double[] v, double[] a, double dt) {
        new DualMotionSegment(new double[]{0, 0}, v, a, 0, dt);
    }

    public DualMotionSegment(double[] p, double[] v, double[] a, double theta, double dt) {
        this.x0 = p[0];
        this.r0 = p[1];
        this.v_t = v[0];
        this.v_r = v[1];
        this.a_t = a[0];
        this.a_r = a[1];
        this.theta = 0;
        this.dt = dt;
    }

    public DualMotionSegment(double v_t, double v_r, double a_t, double a_r, double dt) {
        this(0, 0, v_t, v_r, a_t, a_r, 0, dt);
    }

    public DualMotionSegment(double x0, double r0, double v_t, double v_r, double a_t, double a_r, double theta, double dt) {
        this.x0 = x0;
        this.r0 = r0;
        this.v_t = v_t;
        this.v_r = v_r;
        this.a_t = a_t;
        this.a_r = a_r;
        this.theta = 0;
        this.dt = dt;
    }

    public boolean isDone(double time) {
        return time >= dt;
    }

    public double[] get(double time) {
        return new double[]{v_t * time + .5 * a_t * time * time, v_t + a_t * time, a_t};
    }

    public double[] get_pos(double time) {
        double x = x0 + v_t * time + .5 * a_t * time * time;
        double r = r0 + v_r * time + .5 * a_r * time * time;
        return new double[]{x, r};
    }

    public double[] get_vel(double time) {
        double x = v_t + a_t * time;
        double r = v_r + a_r * time;
        return new double[]{x, r};
    }

    public double[] get_acc(double time) {
        double x = a_t;
        double r = a_r;
        return new double[]{x, r};
    }
}
