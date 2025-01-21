package org.firstinspires.ftc.teamcode.trajectories;

import static org.firstinspires.ftc.teamcode.DanDriveConstants.MAX_ACCELERATION;
import static org.firstinspires.ftc.teamcode.DanDriveConstants.MAX_ROTATIONAL_ACCELERATION;
import static org.firstinspires.ftc.teamcode.DanDriveConstants.MAX_ROTATIONAL_VELOCITY;
import static org.firstinspires.ftc.teamcode.DanDriveConstants.MAX_VELOCITY;
import static org.firstinspires.ftc.teamcode.DanDriveConstants.trackwidth;
import static org.firstinspires.ftc.teamcode.DanDriveConstants.wheelbase;
import static java.lang.Math.abs;
import static java.lang.Math.sqrt;

import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.util.Pose2D;

import java.util.ArrayList;

public class DualLinearMotionProfile implements MotionProfile {

    double pi = Math.PI;
    double lx = wheelbase / 2;
    double ly = trackwidth / 2;

    public Pose2D startPose, endPose, poseEstimate;
    public double p0 = 0;
    public double theta = 0;
    public double sin = 0, cos = 0;
    public double duration = 0;

    boolean x_normal = false;
    boolean r_normal = false;
    boolean x_max_normal = false;
    boolean r_max_normal = false;

    double x_direction = 0;
    double h_direction = 0;

    boolean swapped = false;

    public DualMotionSegment[] trajectory = {};
    public double[][] trajectory2 = {};

    public double[][] getTrajectory() {
        return trajectory2;
    }

    public int currentSegment = 0;
    public ElapsedTime timer;
    public double startTime;
    public double t0;

    Telemetry telemetry;

    TrajectoryCase state = TrajectoryCase.ITS_COMPLICATED;

    public static boolean isTrajNormal(double d, double vmax, double amax) {
        return (vmax / amax) < (d / vmax);
    }

    void add_period(ArrayList<double[]> periods, double x_mult, double r_mult, double dt) {
        if (dt != 0) {
            if (swapped) {
                periods.add(new double[]{MAX_ACCELERATION * r_mult * x_direction, MAX_ROTATIONAL_ACCELERATION * x_mult * h_direction, dt});
            } else {
                periods.add(new double[]{MAX_ACCELERATION * x_mult * x_direction, MAX_ROTATIONAL_ACCELERATION * r_mult * h_direction, dt});
            }
        }

    }

    void fully_degenerate_traj(ArrayList<double[]> periods, double dT, double dt) {

        add_period(periods, 1, 1, dt);
        if (dT > 2*dt) {
            add_period(periods, 1, -1, 2 * dt);
            add_period(periods, 1, 0, dT);
        } else {
            add_period(periods, 1, -1, dT);
            add_period(periods, -1, -1, 2 * dt);
        }
        add_period(periods, -1, 0, 2 * dT);

    }

    public DualLinearMotionProfile(Pose2D start, Pose2D end, Telemetry tel) {

        ArrayList<double[]> periods = new ArrayList<>();

        telemetry = tel;

        startPose = start;
        endPose = end;

        double dh = endPose.h - startPose.h;

        double v_x_c_max = 86.182/2.54;
        double v_r_c_max = MAX_ROTATIONAL_ACCELERATION * (v_x_c_max/MAX_ACCELERATION);

        double dx_x = endPose.x - startPose.x;
        double dx_y = endPose.y - startPose.y;
        double dx = Math.hypot(dx_x, dx_y);
        theta = Math.atan2(dx_y, dx_x);

        cos = Math.cos(theta);
        sin = Math.sin(theta);

        if (Math.abs(dx) < 1e-3) dx = 0;
        if (Math.abs(dh) < 1e-2) dh = 0;

        h_direction = Math.signum(dh);
        dh = Math.abs(dh);
        x_direction = 1;

        x_normal = isTrajNormal(dx, v_x_c_max, MAX_ACCELERATION);
        x_max_normal = isTrajNormal(dx, MAX_VELOCITY, MAX_ACCELERATION);
        r_normal = isTrajNormal(dh, v_r_c_max, MAX_ROTATIONAL_ACCELERATION);
        r_max_normal = isTrajNormal(dh, MAX_ROTATIONAL_VELOCITY, MAX_ROTATIONAL_ACCELERATION);


        p0 = Math.hypot(startPose.x, startPose.y);


        if (!(x_normal || r_normal)){
            
            double dT = sqrt(dx / MAX_ACCELERATION);
            double dt = sqrt(dh / MAX_ROTATIONAL_ACCELERATION);


            if (abs(dT - dt) < 1e-5) {
                add_period(periods, 1, 1, dT);
                add_period(periods, -1, -1, dT);
                return;
            }

            if (dT < dt) {
                double temp = dT;
                dT = dt;
                dt = temp;
                swapped = true;
            }

            fully_degenerate_traj(periods, dT, dt);
            
        } else if (x_normal && !r_normal) {

            double dt = sqrt(dh / MAX_ROTATIONAL_ACCELERATION);
            if (x_max_normal) {
                double dT1 = MAX_VELOCITY / MAX_ACCELERATION;
                double dT2 = (dx / MAX_VELOCITY);

                add_period(periods, 1, 1, dt);

                if (dT1 > 2 * dt) {
                    add_period(periods, 1, -1, 2 * dt);
                    add_period(periods, 1, 0, dT1);
                } else {
                    add_period(periods, 1, -1, dT1);
                    add_period(periods, 0, -1, 2 * dt);
                }
                add_period(periods, 0, 0, dT2);
                add_period(periods, -1, 0, dT1 + dT2);
            } else {
                double dT = sqrt(dx / MAX_ACCELERATION);

                fully_degenerate_traj(periods, dT, dt);
            }


        } else if (!x_normal && r_normal) {
            swapped = true;
            double dt = sqrt(dx / MAX_ACCELERATION);
            if (r_max_normal) {
                double dT1 = MAX_ROTATIONAL_VELOCITY / MAX_ROTATIONAL_ACCELERATION;
                double dT2 = (dh / MAX_ROTATIONAL_VELOCITY);

                add_period(periods, 1, 1, dt);

                if (dT1 > 2 * dt) {
                    add_period(periods, 1, -1, 2 * dt);
                    add_period(periods, 1, 0, dT1);
                }else {
                    add_period(periods, 1, -1, dT1);
                    add_period(periods, 0, -1, 2 * dt);
                }
                add_period(periods, 0, 0, dT2);
                add_period(periods, -1, 0, dT1 + dT2);

            } else {
                double dT = sqrt(dh / MAX_ROTATIONAL_ACCELERATION);

                fully_degenerate_traj(periods, dT, dt);
            }
        }  else {

            double dT1 = v_x_c_max / MAX_ACCELERATION;
            double dT2 = dx / v_x_c_max;

            double dt1 = v_r_c_max / MAX_ROTATIONAL_ACCELERATION;
            double dt2 = dh / v_r_c_max;

            if (dT2 < dt2) {
                swapped = true;
                double temp = dT2;
                dT2 = dt2;
                dt2 = temp;
            }

            add_period(periods, 1, 1, dT1);
            add_period(periods, 0, 0, dt2);

            if (dT2 >= dt1 + dt2) {
                add_period(periods, 0, -1, dt1 + dt2);

                // somehow make it reaccelerate
                // I MADE IT REACCELERATEEEEEEEEEEEE

                if (!swapped) {

                    double dx2 = dx - v_x_c_max * dt2;

                    if (isTrajNormal(dx2, MAX_VELOCITY, MAX_ACCELERATION)) {

                        double dT12 = MAX_VELOCITY / MAX_ACCELERATION;
                        double dT22 = dx2 / MAX_VELOCITY;

                        add_period(periods, 1, 0, dt2 + dT12);
                        add_period(periods, 0, 0, dt2 + dT22);
                        add_period(periods, -1, 0, 2.0);

                    } else {

                        double dT = sqrt(dx2 / MAX_ACCELERATION);

                        add_period(periods, 1, 0, dT + dt2);
                        add_period(periods, -1, 0, dT + dT + dt2);
                    }

                } else {

                    double h2 = dh - v_r_c_max * dt2;

                    if (isTrajNormal(h2, MAX_ROTATIONAL_VELOCITY, MAX_ROTATIONAL_ACCELERATION)) {

                        double dT12 = MAX_ROTATIONAL_VELOCITY / MAX_ROTATIONAL_ACCELERATION;
                        double dT22 = h2 / MAX_ROTATIONAL_VELOCITY;

                        add_period(periods, 1, 0, dt2 + dT12);
                        add_period(periods, 0, 0, dt2 + dT22);
                        add_period(periods, -1, 0, 2.0);

                    } else {

                        double dT = sqrt(h2 / MAX_ROTATIONAL_ACCELERATION);

                        add_period(periods, 1, 0, dT + dt2);
                        add_period(periods, -1, 0, dT + dT + dt2);
                    }

                }
            } else {
                add_period(periods, 0, -1, dT2);
                add_period(periods, -1, -1, dt1 + dt2);

                add_period(periods, -1, 0, dT1 + dT2);
            }
        }

        ArrayList<DualMotionSegment> collection = new ArrayList<>();

        double t0 = 0;
        double x0 = 0;
        double h0 = 0;
        double vx0 = 0;
        double vh0 = 0;

        for (double[] period : periods) {

            double dt = period[2] - t0;

            collection.add(
                    new DualMotionSegment(
                            x0, h0, vx0, vh0, period[0], period[1], theta, dt
                    )
            );

            x0 += vx0 * dt + .5 * period[0] * dt * dt;
            vx0 += period[0] * dt;

            h0 += vh0 * dt + .5 * period[1] * dt * dt;
            vh0 += period[1] * dt;

            t0 = period[2];
        }

        trajectory = collection.toArray(new DualMotionSegment[periods.size()]);
        trajectory2 = periods.toArray(new double[0][]);
        duration = t0;

    }

    public DualLinearMotionProfile(Pose2D end, Telemetry t) {
        new DualLinearMotionProfile(new Pose2D(0, 0, 0), end, t);
    }

    public void start() {
        timer = new ElapsedTime();
        startTime = 0;
    }

    public Pose2D[] get_time() {

        double time = timer.time() - startTime;
        if (time >= duration) {
            return new Pose2D[]{
                    endPose,
                    new Pose2D(0, 0, 0),
                    new Pose2D(0, 0, 0)
            };
        }
        double x = Math.hypot(startPose.x, startPose.y);
        double h = startPose.h;
        double[] v = {0, 0};
        double[] a = {0, 0};

        for (int i = 0; i < trajectory2.length; i++) {

            double[] segment = trajectory2[i];

            double dt = segment[2] - ((i > 0) ? trajectory2[i-1][2] : 0);

            if (time < segment[2]) {

                time -= ((i > 0) ? trajectory2[i-1][2] : 0);

                x += v[0] * time + .5 * segment[0] * time * time;
                h += v[1] * time + .5 * segment[1] * time * time;
                v[0] += segment[0] * time;
                v[1] += segment[1] * time;
                a[0] = segment[0];
                a[1] = segment[1];
                break;

            }
            x += v[0] * dt + .5 * segment[0] * dt * dt;
            h += v[1] * dt + .5 * segment[1] * dt * dt;
            v[0] += segment[0] * dt;
            v[1] += segment[1] * dt;
            a[0] = segment[0];
            a[1] = segment[1];
        }

        double theta = this.theta - h;
        return new Pose2D[]{
                new Pose2D(x * cos, x * sin, h),
                new Pose2D(v[0] * cos, v[0] * sin, v[1]),
                new Pose2D(a[0] * cos, a[0] * sin, a[1])
        };
    }

    public Pose2D traj_pos_time() {
        double time = timer.time() - t0;
        double x = this.p0;
        double r = startPose.h;

        DualMotionSegment segment = trajectory[currentSegment];

        if (time > segment.dt) {
            t0 += segment.dt;
            currentSegment++;
            segment = trajectory[currentSegment];
        }

        time = timer.time() - t0;
        double[] p = segment.get_pos(time);
        x += p[0]; r += p[1];

        return new Pose2D(x * cos, x * sin, r);
    }

    public Pose2D traj_vel_time() {
        double time = timer.time() - t0;
        double[] v = {0, 0};

        DualMotionSegment segment = trajectory[currentSegment];

        if (time > segment.dt) {
            t0 += segment.dt;
            currentSegment++;
            segment = trajectory[currentSegment];
        }

        v = segment.get_vel(time);

        double a = theta - traj_pos_time().h;
        return new Pose2D(v[0] * Math.cos(a), v[0] * Math.sin(a), v[1]);
    }

    public Pose2D traj_acc_time() {
        double time = timer.time() - t0;
        double[] a = {0, 0};

        DualMotionSegment segment = trajectory[currentSegment];

        if (time > segment.dt) {
            t0 += segment.dt;
            currentSegment++;
            segment = trajectory[currentSegment];
        }

        time = timer.time() - t0;
        a = segment.get_acc(time);

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
