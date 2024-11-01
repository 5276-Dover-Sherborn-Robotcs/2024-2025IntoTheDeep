package org.firstinspires.ftc.teamcode.trajectories;

import static org.firstinspires.ftc.teamcode.DriveConstants.MAX_ACCELERATION;
import static org.firstinspires.ftc.teamcode.DriveConstants.MAX_VELOCITY;

import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;

public class LinearMotionProfile extends MotionProfile {

    public Pose2D startPose, endPose;
    public double p0 = 0;
    public double d = 0;
    public double theta = 0;
    public double sin = 0, cos = 0;

    public MotionSegment[] trajectory;

    public ElapsedTime timer;
    public double t0;

    public LinearMotionProfile(Pose2D startPose, Pose2D endPose) {

        this.startPose = startPose;
        this.endPose = endPose;

        double dx = endPose.getX(DistanceUnit.CM) - startPose.getX(DistanceUnit.CM);
        double dy = endPose.getY(DistanceUnit.CM) - startPose.getY(DistanceUnit.CM);
        d = Math.sqrt(dx * dx + dy * dy);
        theta = Math.atan2(dx, dy);
        sin = Math.sin(theta - Math.PI / 4);
        cos = Math.cos(theta - Math.PI / 4);

        if (MAX_VELOCITY / MAX_ACCELERATION < d / MAX_VELOCITY) {
            // normal trajectory
            double dt1 = MAX_VELOCITY / MAX_ACCELERATION;
            double dt2 = dx / MAX_VELOCITY - MAX_VELOCITY / MAX_ACCELERATION;
            trajectory = new MotionSegment[]{
                    new MotionSegment(0, MAX_ACCELERATION, dt1),
                    new MotionSegment(MAX_VELOCITY, 0, dt2, dt1),
                    new MotionSegment(MAX_VELOCITY, -MAX_ACCELERATION, dt1, dt1 + dt2),
            };
        } else {
            // degenerate trajectory
            double dt1 = Math.sqrt(dx / MAX_ACCELERATION);
            trajectory = new MotionSegment[]{
                    new MotionSegment(0, MAX_ACCELERATION, dt1),
                    new MotionSegment(MAX_ACCELERATION*dt1, MAX_ACCELERATION, dt1, dt1)
            };
        }
    }

    public void start() {
        t0 = timer.time();
    }

    public Pose2D traj_pos_time() {
        double time = timer.time() - t0;
        double x0 = this.p0;
        for (MotionSegment segment : trajectory) {
            if (!segment.isDone(time)) {
                x0 += segment.get_pos(time);
                break;
            }
            time -= segment.dt;
            x0 += segment.x1;
        }
        return new Pose2D(DistanceUnit.CM, x0 * cos, x0 * sin, AngleUnit.RADIANS, 0);
    }

    public Pose2D traj_vel_time() {
        double time = timer.time() - t0;
        double v0 = 0;
        for (MotionSegment segment : trajectory) {
            if (!segment.isDone(time)) {
                v0 += segment.get_vel(time);
                break;
            }
            time -= segment.dt;
            v0 += segment.v1;
        }
        return new Pose2D(DistanceUnit.CM, v0 * cos, v0 * sin, AngleUnit.RADIANS, 0);
    }

    public Pose2D traj_acc_time() {
        double time = timer.time() - t0;
        double a = 0;
        for (MotionSegment segment : trajectory) {
            if (!segment.isDone(time)) {
                a = segment.a; break;
            }
            time -= segment.dt;
        }
        return new Pose2D(DistanceUnit.CM, a * cos, a * sin, AngleUnit.RADIANS, 0);
    }

    public boolean is_traj_done() {
        double time = timer.time() - t0;
        return trajectory[trajectory.length - 1].isDone(time);
    }

    public void end() {
        sin = 0;
        cos = 0;
        p0 = 0;
        d = 0;
        t0 = 0;
        trajectory = null;
        timer = null;
    }

}
