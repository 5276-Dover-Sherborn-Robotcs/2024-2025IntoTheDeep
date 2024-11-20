package org.firstinspires.ftc.teamcode.trajectories;

import static org.firstinspires.ftc.teamcode.DriveConstants.MAX_ACCELERATION;
import static org.firstinspires.ftc.teamcode.DriveConstants.MAX_VELOCITY;

import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;

public class LinearMotionProfile extends MotionProfile {

    public Pose2D startPose, endPose;
    public double p0 = 0;
    public double d = 0;
    public double theta = 0;
    public double sin = 0, cos = 0;
    public double duration = 0;

    public MotionSegment[] trajectory = {};

    public ElapsedTime timer;
    public double t0;

    Telemetry telemetry;

    public LinearMotionProfile(Pose2D startPose, Pose2D endPose, Telemetry telemetry) {

        this.telemetry = telemetry;

        this.startPose = startPose;
        this.endPose = endPose;

        double dx = endPose.getX(DistanceUnit.CM) - startPose.getX(DistanceUnit.CM);
        double dy = endPose.getY(DistanceUnit.CM) - startPose.getY(DistanceUnit.CM);
        d = Math.hypot(dx, dy);
        theta = Math.atan2(dy, dx);
        sin = Math.sin(theta);
        cos = Math.cos(theta);

        if (MAX_VELOCITY / MAX_ACCELERATION < d / MAX_VELOCITY) {
            state = TrajectoryCase.NORMAL;
            // normal trajectory
            double dt1 = MAX_VELOCITY / MAX_ACCELERATION;
            double dt2 = d / MAX_VELOCITY - MAX_VELOCITY / MAX_ACCELERATION;
            trajectory = new MotionSegment[]{
                    new MotionSegment(0, MAX_ACCELERATION, dt1),
                    new MotionSegment(.5*MAX_ACCELERATION*dt1*dt1, MAX_VELOCITY, 0, dt2),
                    new MotionSegment(MAX_VELOCITY, -MAX_ACCELERATION, dt1),
            };
            duration = 2 * dt1 + dt2;
        } else {
            state = TrajectoryCase.DEGENERATE;
            // degenerate trajectory
            double dt1 = Math.sqrt(d / MAX_ACCELERATION);
            trajectory = new MotionSegment[]{
                    new MotionSegment(0, MAX_ACCELERATION, dt1),
                    new MotionSegment(d/2, MAX_ACCELERATION*dt1, -MAX_ACCELERATION, dt1)
            };
            duration = 2 * dt1;
        }
    }

    public void start() {
        timer = new ElapsedTime();
    }

    public Pose2D traj_pos_time() {
        double time = timer.time();
        double x0 = this.p0;
        for (MotionSegment segment : trajectory) {
            if (time < segment.dt) {
                x0 += segment.get_pos(time);
                break;
            }
            time -= segment.dt;
            x0 += segment.get_pos(segment.dt);
        }
        telemetry.addData("Target Position", x0);
        return new Pose2D(DistanceUnit.CM, x0 * cos, x0 * sin, AngleUnit.RADIANS, theta);
    }

    public Pose2D traj_vel_time() {
        double time = timer.time();
        double v0 = 0;
        for (MotionSegment segment : trajectory) {
            if (time < segment.dt) {
                v0 = segment.get_vel(time);
                break;
            }
            time -= segment.dt;
        }
        telemetry.addData("Target Velocity", v0);
        return new Pose2D(DistanceUnit.CM, v0 * cos, v0 * sin, AngleUnit.RADIANS, theta);
    }

    public Pose2D traj_acc_time() {
        double time = timer.time();
        double a = 0;
        for (MotionSegment segment : trajectory) {
            if (time < segment.dt) {
                a = segment.a;
                break;
            }
            time -= segment.dt;
        }
        telemetry.addData("Target Acceleration", a);
        double max = Math.max(Math.abs(sin), Math.abs(cos));
        return new Pose2D(DistanceUnit.CM, a * cos/max, a * sin/max, AngleUnit.RADIANS, theta);
    }

    public boolean is_traj_done() {
        double time = timer.time();
        for (MotionSegment segment : trajectory) {
            time -= segment.dt;
        }
        return time >= 0;
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

    public TrajectoryCase getState() {
        return state;
    }

    public void telemetrize() {
        telemetry.addData("Theta", theta);
        telemetry.addData("Duration", duration);
    }

}
