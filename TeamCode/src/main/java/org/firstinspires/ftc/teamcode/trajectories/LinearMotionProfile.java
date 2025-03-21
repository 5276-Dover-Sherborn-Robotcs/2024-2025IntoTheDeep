package org.firstinspires.ftc.teamcode.trajectories;

import static org.firstinspires.ftc.teamcode.DanDriveConstants.MAX_ACCELERATION;
import static org.firstinspires.ftc.teamcode.DanDriveConstants.MAX_VELOCITY;

import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.util.Pose2D;

public class LinearMotionProfile implements MotionProfile {

    public Pose2D startPose, endPose;
    public double d = 0;
    public double theta = 0;
    public double sin = 0, cos = 0;
    public double duration = 0;

    public MotionSegment[] trajectory = {};

    public ElapsedTime timer;

    Telemetry telemetry;

    TrajectoryCase state;

    public LinearMotionProfile(Pose2D startPose, Pose2D endPose, Telemetry telemetry) {

        this.telemetry = telemetry;

        this.startPose = startPose;
        this.endPose = endPose;

        double dx = endPose.x - startPose.x;
        double dy = endPose.y - startPose.y;
        d = Math.hypot(dx, dy);
        theta = Math.atan2(dy, dx);
//        theta = Math.atan2(dy, dx) + Math.PI/4;
        sin = Math.sin(theta);
        cos = Math.cos(theta);

//        double d_heading = endPose.getHeading(AngleUnit.RADIANS) - startPose.getHeading(AngleUnit.RADIANS);
//        if (MAX_ROTATIONAL_VELOCITY / MAX_ROTATIONAL_ACCELERATION < d_heading / MAX_ROTATIONAL_VELOCITY) {
//            double dt1 = MAX_ROTATIONAL_VELOCITY / MAX_ROTATIONAL_ACCELERATION;
//            double dt2 = d_heading / MAX_ROTATIONAL_VELOCITY - dt1;
//        }

        if (MAX_VELOCITY / MAX_ACCELERATION < d / MAX_VELOCITY) {
            state = TrajectoryCase.NORMAL;
            // normal trajectory
            double dt1 = MAX_VELOCITY / MAX_ACCELERATION;
            double dt2 = d / MAX_VELOCITY - dt1;
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

    public double[][] getTrajectory() {
        return new double[0][];
    }

    @Override
    public Pose2D[] getEverything() {

        double time = timer.time();
        double x0 = 0;
        double v0 = 0;
        double a = 0;
        for (MotionSegment segment : trajectory) {
            if (time < segment.dt) {
                x0 += segment.get_pos(time);
                v0 = segment.get_vel(time);
                a = segment.a;
                break;
            }
            time -= segment.dt;
            x0 += segment.get_pos(segment.dt);
        }

        return new Pose2D[]{
                new Pose2D(x0 * cos, x0 * sin, 0),
                new Pose2D(v0 * cos, v0 * sin, 0),
                new Pose2D(a * cos, a * sin, 0)
        };

    }

    public Pose2D trajectoryPosition() {
        double time = timer.time();
        double x0 = 0;
        for (MotionSegment segment : trajectory) {
            if (time < segment.dt) {
                x0 += segment.get_pos(time);
                break;
            }
            time -= segment.dt;
            x0 += segment.get_pos(segment.dt);
        }
        telemetry.addData("Target Position", x0);
        return new Pose2D(x0 * cos, x0 * sin, 0);
    }

    public Pose2D trajectoryVelocity() {
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
        return new Pose2D(v0 * cos, v0 * sin, 0);
    }

    public Pose2D trajectoryAcceleration() {
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
        return new Pose2D(a * cos, a * sin, 0);
    }

    public boolean is_traj_done() {
        double time = timer.time();
        for (MotionSegment segment : trajectory) {
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
