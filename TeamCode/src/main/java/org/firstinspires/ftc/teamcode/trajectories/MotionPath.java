package org.firstinspires.ftc.teamcode.trajectories;

import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;

public class MotionPath {

    public Pose2D startPose;
    public Pose2D endPose;

    public double duration;

    public MotionSegment[] segments;

    public MotionPath(MotionSegment[] list) {
        segments = list;
    }

    public MotionSegment[] getSegments() {
        return segments;
    }

    public Pose2D getStartPose() {
        return startPose;
    }

    public Pose2D getEndPose() {
        return endPose;
    }

    public static class MotionPathBuilder {

        private final Pose2D startPose;
        private Pose2D endPose;
        private Pose2D poseEstimate;
        private Pose2D velocityEstimate;

        public MotionPathBuilder(Pose2D start) {
            this.startPose = start;
            this.poseEstimate = start;
        }

//        public MotionPathBuilder lineTo(Pose2D end) {
//
//        }

    }

}
