package org.firstinspires.ftc.teamcode;


import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.util.Pose2D;

@Autonomous(name="Just Moving test Autonomous")
public class JustMovingAuto extends AutonomousOpMode {

    // Rotation positions, mostly just for a level of abstraction. Following that are a couple pid used things
    public enum rotations implements Rotation {
        IDLE(0);

        public final double rotation;

        rotations(double rotation) {this.rotation = rotation;}
    }

    //Extension positions, in inches
    public enum extensions implements Extension{
        IDLE(0);

        public final double extension;

        extensions(double extension) {this.extension = extension;}

    }

    // Positions the robot wants to be at
    public enum positions implements Position {
        POS_1(new Pose2D(12, 0, Math.PI/2)),
        POS_2(new Pose2D(12, 12, Math.PI)),
        POS_3(new Pose2D(0, 12, 3*Math.PI/2));

        public final Pose2D pose;

        positions(Pose2D pose) {this.pose = pose;}
    }
    // The order of positions the robot wants to move to

    @Override
    public void variable_init() {

        target_positions = new Position[]{
                positions.POS_1,
                positions.POS_2,
                positions.POS_3
        };

        target_extension = extensions.IDLE;
        target_rotation = rotations.IDLE;
        startPose = new Pose2D(0, 0, 0);

    }

    @Override
    public void mainLoop() {

        switch (state) {
            case IDLE:
                idle();
            case MOVING:
                if (profile_done) {
                    target_position_index++;

                    if (target_position_index == target_positions.length) state = states.IDLE;
                }
                break;

        }

    }

}
