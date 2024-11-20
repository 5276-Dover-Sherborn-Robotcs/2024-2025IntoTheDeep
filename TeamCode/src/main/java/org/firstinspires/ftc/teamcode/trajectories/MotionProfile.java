package org.firstinspires.ftc.teamcode.trajectories;

import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;

public abstract class MotionProfile {

    public enum TrajectoryCase {
        NORMAL,
        DEGENERATE
    }

    public TrajectoryCase state = TrajectoryCase.NORMAL;
    public abstract void start();
    public abstract void end();
    public abstract Pose2D traj_pos_time();
    public abstract Pose2D traj_vel_time();
    public abstract Pose2D traj_acc_time();
    public abstract boolean is_traj_done();
}
