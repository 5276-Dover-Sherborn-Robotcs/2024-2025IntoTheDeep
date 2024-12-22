package com.example.meepmeeptesting;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.util.NanoClock;

import org.rowlandhall.meepmeep.MeepMeep;
import org.rowlandhall.meepmeep.roadrunner.DefaultBotBuilder;
import org.rowlandhall.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

public class MeepMeepTesting {
    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(1200);

        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(87.0, 87/2.0, Math.toRadians(180), Math.toRadians(180), 15)
                .followTrajectorySequence(drive -> drive.trajectorySequenceBuilder(new Pose2d(-24, -72+15/2.0, Math.toRadians(90)))
                        .splineToLinearHeading(new Pose2d(-48, -60, Math.toRadians(190)), Math.toRadians(270))
                        .lineToLinearHeading(new Pose2d(-36, -36, Math.toRadians(135)))
                        .lineToLinearHeading(new Pose2d(-48, -60, Math.toRadians(190)))
                        .lineToLinearHeading(new Pose2d(-48, -36, Math.toRadians(135)))
                        .lineToLinearHeading(new Pose2d(-48, -60, Math.toRadians(190)))
                        .lineToLinearHeading(new Pose2d(-48, -24, Math.toRadians(180)))
                        .lineToLinearHeading(new Pose2d(-48, -60, Math.toRadians(190)))
                        .build());


        meepMeep.setBackground(MeepMeep.Background.FIELD_INTOTHEDEEP_JUICE_DARK)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }

    public static void sleepy() {

        double t0 = NanoClock.system().seconds();

        while (NanoClock.system().seconds() - t0 < 2) {
            continue;
        }

    }

}