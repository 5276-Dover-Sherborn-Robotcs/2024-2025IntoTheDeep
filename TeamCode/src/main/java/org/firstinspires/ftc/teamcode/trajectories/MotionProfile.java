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
    Pose2D[] getEverything();
    Pose2D trajectoryPosition();
    Pose2D trajectoryVelocity();
    Pose2D trajectoryAcceleration();
    boolean is_traj_done();


}
