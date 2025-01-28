package org.firstinspires.ftc.teamcode.trajectories;

import org.firstinspires.ftc.teamcode.util.Pose2D;

public interface MotionProfile {

    enum TrajectoryCase {
        NORMAL,
        DEGENERATE,
        ITS_COMPLICATED
    }

    double[][] getTrajectory();
    void start();
    void end();
    Pose2D[] get_state_at_time();
    Pose2D traj_pos_time();
    Pose2D traj_vel_time();
    Pose2D traj_acc_time();
    boolean is_traj_done();


}
