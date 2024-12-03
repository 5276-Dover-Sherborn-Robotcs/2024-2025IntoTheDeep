package org.firstinspires.ftc.teamcode.trajectories;

import static org.firstinspires.ftc.teamcode.DriveConstants.MAX_ACCELERATION;
import static org.firstinspires.ftc.teamcode.DriveConstants.MAX_ROTATIONAL_ACCELERATION;
import static org.firstinspires.ftc.teamcode.DriveConstants.MAX_VELOCITY;
import static org.firstinspires.ftc.teamcode.DriveConstants.trackwidth;
import static org.firstinspires.ftc.teamcode.DriveConstants.wheelbase;
import static java.lang.Math.abs;
import static java.lang.Math.max;

import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.util.Pose2D;

import java.util.ArrayList;
import java.util.List;

public class DualLinearMotionProfile extends MotionProfile {

    double pi = Math.PI;
    double lx = wheelbase / 2;
    double ly = trackwidth / 2;

    public Pose2D startPose, endPose;
    public double p0 = 0;
    public double theta = 0;
    public double sin = 0, cos = 0;
    public double duration = 0;

    public DualMotionSegment[] trajectory = {};

    public ElapsedTime timer;
    public double t0;

    Telemetry telemetry;

    public DualLinearMotionProfile(Pose2D start, Pose2D end, Telemetry tel) {

        telemetry = tel;

        startPose = start;
        endPose = end;

        double dh = endPose.h - startPose.h;
        double h_direction = Math.signum(dh);

        double dx_x = endPose.x - startPose.x;
        double dx_y = endPose.y - startPose.y;
        double dx = Math.hypot(dx_x, dx_y);
        theta = Math.atan2(dx_y, dx_x);
        double x_direction = Math.signum(dx);

        double x = 0;
        double h = 0;
        double t = 0;
        double dt = 0.0005;
        double v_t= 0;
        double v_r = 0;
        double a_t = MAX_ACCELERATION;
        double a_r = MAX_ROTATIONAL_ACCELERATION;
        double dx_a = 0;
        double dh_a = 0;

        int c = 0;

        List<Double[]> positions = new ArrayList<Double[]>();
        List<Double[]> velocities = new ArrayList<Double[]>();
        List<Double[]> accelerations = new ArrayList<Double[]>();

        boolean arrived_x = abs(dx - x) <= 0.01;
        boolean arrived_r = abs(dh - h) <= pi/360;
        boolean completed_first = false;

        boolean constraint = false;
        List<Integer> constrains = new ArrayList<Integer>();

        List<DualMotionSegment> periods = new ArrayList<DualMotionSegment>();

        while (!arrived_x || !arrived_r) {

            double olda_t = a_t;
            double olda_r = a_r;

            arrived_x = abs(dx - x) <= 0.01;
            arrived_r = abs(dh - h) <= pi/360;

            dx_a = max(dx_a, -(v_t*v_t) / (2 * -MAX_ACCELERATION));

            dh_a = max(dh_a, -(v_r*v_r) / (2 * -MAX_ROTATIONAL_ACCELERATION));

            if (abs(dx - x) <= dx_a) {
                a_t = -MAX_ACCELERATION;
                if (arrived_x) {
                    a_t = 0;
                    v_t = 0;
                    if (a_r == 0 && !arrived_r && !completed_first) {
                        a_r = MAX_ROTATIONAL_ACCELERATION;
                        constraint = false;
                        completed_first = true;
                    }
                }
            }
            if (abs(dh - h) <= dh_a) {
                a_r = -MAX_ROTATIONAL_ACCELERATION;
                if (arrived_r) {
                    a_r = 0;
                    v_r = 0;
                    if (a_t == 0 && !arrived_x && !completed_first) {
                        a_r = MAX_ACCELERATION;
                        constraint = false;
                        completed_first = true;
                    }
                }
            }

            if (constrained(v_t, v_r, h) && !constraint) {
                a_t = 0;
                a_r = 0;
                constraint = true;
                constrains.add(c);
            }

            positions.add(new Double[]{x, h});
            velocities.add(new Double[]{v_t, v_r});
            accelerations.add(new Double[]{a_t, a_r});

            if (periods.isEmpty()) {
                periods.add(new DualMotionSegment(x, h, v_t, v_r, a_t, a_r, theta, 0));
            } else if (olda_r != a_r || olda_t != a_t) {
                periods.get(periods.size()-1).dt = t - periods.get(periods.size()-1).dt;
                periods.add(new DualMotionSegment(x, h, v_t, v_r, a_t, a_r, theta, t));
            }

            x += v_t * dt + .5 * a_t * dt * dt;
            if ((MAX_ACCELERATION - v_t) <= dt * a_t) {
                a_t = 0;
                v_t = MAX_ACCELERATION * x_direction;
            } else {
                v_t += a_t * dt * x_direction;
            }

            h += v_r * dt + .5 * a_r * dt * dt;
            if ((MAX_ROTATIONAL_ACCELERATION - v_r) <= dt * a_r) {
                a_r = 0;
                v_r = MAX_ROTATIONAL_ACCELERATION * h_direction;
            } else{
                v_r += a_r * dt * h_direction;
            }

            t += dt;

            c++;

        }

        periods.get(periods.size()-1).dt = t;

        duration = t;

        trajectory = periods.toArray(trajectory);

    }

    public boolean constrained(double v, double r, double h) {
        double a = theta - h - pi/4 - (-h % (pi/2) - pi/4);
        double x = Math.cos(a) * v;
        double y = Math.sin(a) * v;
        double w = (lx + ly) * r;
        double[] vels = {
                x - w,
                y + w,
                y - w,
                x + w
        };
        for (double vel : vels) {
            if (abs(vel) >= MAX_VELOCITY) return true;
        }
        return true;
    }

    public DualLinearMotionProfile(Pose2D end, Telemetry t) {
        new DualLinearMotionProfile(new Pose2D(0, 0, 0), end, t);
    }

    public void start() {
        timer = new ElapsedTime();
    }

    public Pose2D[] get_time() {
        double time = timer.time();
        double x0 = Math.hypot(startPose.x, startPose.y);
        double r0 = startPose.h;
        double[] v = {0, 0};
        double[] a = {0, 0};
        for (DualMotionSegment segment: trajectory) {
            if (time < segment.dt) {
                x0 += segment.get_pos(time)[0];
                r0 += segment.get_pos(time)[1];
                v = segment.get_vel(time);
                a = segment.get_acc(time);
                break;
            }
            time -= segment.dt;
            x0 += segment.get_pos(segment.dt)[0];
            r0 += segment.get_pos(segment.dt)[1];
            v = segment.get_vel(segment.dt);
            a = segment.get_acc(segment.dt);
        }
        double h = theta - r0;
        return new Pose2D[]{
                new Pose2D(x0 * cos, x0 * sin, r0),
                new Pose2D(v[0] * Math.cos(h), v[0] * Math.sin(h), v[1]),
                new Pose2D(a[0] * Math.cos(h), a[0] * Math.sin(h), a[1])
        };
    }

    public Pose2D traj_pos_time() {
        double time = timer.time();
        double x0 = this.p0;
        double r0 = startPose.h;
        for (DualMotionSegment segment : trajectory) {
            if (time < segment.dt) {
                x0 += segment.get_pos(time)[0];
                r0 += segment.get_pos(time)[1];
                break;
            }
            time -= segment.dt;
            x0 += segment.get_pos(segment.dt)[0];
            r0 += segment.get_pos(segment.dt)[1];
        }
        telemetry.addData("Target Position", x0);
        return new Pose2D(x0 * cos, x0 * sin, r0);
    }

    public Pose2D traj_vel_time() {
        double time = timer.time();
        double[] v = null;
        for (DualMotionSegment segment : trajectory) {
            if (time < segment.dt) {
                v = segment.get_vel(time);
                break;
            }
            time -= segment.dt;
            v = segment.get(segment.dt);
        }
        telemetry.addData("Target Velocity", v);
        double a = theta - traj_pos_time().h;
        return new Pose2D(v[0] * Math.cos(a), v[0] * Math.sin(a), v[1]);
    }

    public Pose2D traj_acc_time() {
        double time = timer.time();
        double[] a = {0, 0};
        for (DualMotionSegment segment : trajectory) {
            if (time < segment.dt) {
                a = new double[]{segment.a_t, segment.a_r};
                break;
            }
            time -= segment.dt;
        }
        telemetry.addData("Target Acceleration", a);
        double h = theta - traj_pos_time().h;
        return new Pose2D(a[0] * Math.cos(h), a[0] * Math.sin(h), a[1]);
    }

    public boolean is_traj_done() {
        double time = timer.time();
        for (DualMotionSegment segment : trajectory) {
            time -= segment.dt;
        }
        return time >= 0;
    }

    public void end() {
        timer = null;
    }

    public TrajectoryCase getState() {
        return state;
    }

    public void telemetrize() {
        telemetry.addData("Theta", theta);
        telemetry.addData("Duration", duration);
    }

}
