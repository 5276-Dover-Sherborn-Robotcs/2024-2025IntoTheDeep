package org.firstinspires.ftc.teamcode.trajectories;

import org.firstinspires.ftc.teamcode.util.Pose2D;

import java.util.ArrayList;

public class MotionPath {

    public Pose2D startPose;
    public Pose2D endPose;

    public double duration;

    public MotionProfile[] profiles;

    public double getDuration() {
        return duration;
    }

    public MotionProfile[] getProfiles() {
        return profiles;
    }

    public Pose2D getStartPose() {
        return startPose;
    }

    public Pose2D getEndPose() {
        return endPose;
    }

    public MotionPath(MotionProfile[] profiles, Pose2D startPose, Pose2D endPose, double duration) {
        this.startPose = startPose;
        this.endPose = endPose;
        this.profiles = profiles;
        this.duration = duration;
    }

    public MotionPath(MotionPathBuilder builder) {
        this.startPose = builder.start;
        this.endPose = builder.end;
        this.duration = builder.time;
        this.profiles = builder.profiles.toArray(new MotionProfile[0]);
    }

    public static class MotionPathBuilder {

        private double time = 0;
        private final Pose2D start;
        private Pose2D end;
        private Pose2D estimate;

        private final ArrayList<MotionProfile> profiles = new ArrayList<>();

        public MotionPathBuilder(Pose2D start) {
            this.start = start;
            this.estimate = start;
        }

        public MotionPathBuilder lineTo(Pose2D end) {

            DualLinearMotionProfile profile = new DualLinearMotionProfile(
                    estimate,
                    end
            );

            profiles.add(profile);
            time += profile.duration;

            estimate = end;

            return this;

        }

        public MotionPathBuilder hold(double time) {

            DualLinearMotionProfile profile = new DualLinearMotionProfile(
                    estimate
            );

            profiles.add(profile);
            this.time += time;

            return this;

        }

        public MotionPath build() {

            end = estimate;

            if (!profiles.isEmpty()) {
                return new MotionPath(this);
            }
            return null;
        }

    }

    public int current_profile = 0;

    public boolean running = false;
    public boolean finished = false;

    public MotionProfile getCurrentProfile() {
        return profiles[current_profile];
    }

    public void start() {
        running = true;
        profiles[0].start();
    }

    public void next() {
        profiles[current_profile].end();
        if (current_profile < profiles.length - 1) {
            current_profile++;
            profiles[current_profile].start();
        }
        else finished = true;
    }

    public Pose2D getTargetPosition() {
        if (finished) return endPose;
        else if (!running) return startPose;
        else {
            return profiles[current_profile].trajectoryPosition();
        }
    }

    public Pose2D getTargetVelocity() {
        if (finished) return endPose;
        else if (!running) return startPose;
        else {
            return profiles[current_profile].trajectoryVelocity();
        }
    }

    public Pose2D getTargetAcceleration() {
        if (finished) return endPose;
        else if (!running) return startPose;
        else {
            return profiles[current_profile].trajectoryAcceleration();
        }
    }

    public Pose2D[] getEverything() {
        if (finished) return new Pose2D[]{endPose, new Pose2D(0, 0, 0), new Pose2D(0, 0, 0)};
        else if (!running) return new Pose2D[]{startPose, new Pose2D(0, 0, 0), new Pose2D(0, 0, 0)};
        else {
            return profiles[current_profile].getEverything();
        }
    }

}
