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

public class DualLinearMotionProfile extends MotionProfile {

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

    boolean swapped = false;

    public DualMotionSegment[] trajectory = {};

    public ElapsedTime timer;
    public double t0;

    Telemetry telemetry;

    void add_period(ArrayList<double[]> periods, double x_mult, double r_mult, double dt) {
        if (dt != 0) {
            if (swapped) {
                periods.add(new double[]{MAX_ACCELERATION * x_mult, MAX_ROTATIONAL_ACCELERATION * r_mult, dt});
            } else {
                periods.add(new double[]{MAX_ACCELERATION * r_mult, MAX_ROTATIONAL_ACCELERATION * x_mult, dt});
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
        double h_direction = Math.signum(dh);

        double v_x_c_max = MAX_VELOCITY / 2;
        double v_r_c_max = MAX_ROTATIONAL_VELOCITY / 2;

        double dx_x = endPose.x - startPose.x;
        double dx_y = endPose.y - startPose.y;
        double dx = Math.hypot(dx_x, dx_y);
        theta = Math.atan2(dx_y, dx_x);


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
        } else {

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

            add_period(periods, 0, -1, dt1 + dt2);

            add_period(periods, 0, 0, dT2);
            add_period(periods, -1, 0, dT1 + dT2);
        }



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
