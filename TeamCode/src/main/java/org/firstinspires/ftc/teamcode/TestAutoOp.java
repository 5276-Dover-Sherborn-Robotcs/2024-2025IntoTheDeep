package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.util.Pose2D;

@Autonomous(name="Test Autonomous mode")
public class TestAutoOp extends AutonomousOpMode {

    public enum position implements Position {
        IDLE(new Pose2D(0, 0, 0)),
        NEXT(new Pose2D(12 * Math.sqrt(3), 12, Math.PI/4)),
        FINAL(new Pose2D(0, 24, 0));

        public final Pose2D pose;

        position(Pose2D pose) {
            this.pose = pose;
        }

        public Pose2D getPose() {
            return pose;
        }
    }
    public enum rotation implements Rotation {
        IDLE(0),
        UP(65.0),
        UP2(90);

        public final double rot;

        rotation(double rot) {
            this.rot = rot;
        }

        public double getRotation() {
            return rot;
        }
    }
    public enum extension implements Extension {
        IDLE(0.0),
        OUT(20);

        public final double ext;

        extension(double ext) {
            this.ext = ext;
        }

        public double getExtension() {
            return ext;
        }
    }

    public rotation[] rotations = {
            rotation.IDLE,
            rotation.UP,
            rotation.UP2
    };
    public int rotation_index = 0;

    double start_time = 0;

    boolean swapped_recently = false;
    boolean we_extended = false;
    boolean we_retracted = false;

    @Override
    public void mainLoop() {

        double time = clock.seconds() - start_time;

        if (time > 10 && profile_done) {

            target_position_index = (target_position_index + 1) % 3;

            start_time = clock.seconds();

        }

    }

    @Override
    public void variable_init() {

        target_rotation = rotation.IDLE;
        target_extension = extension.IDLE;
        target_positions = new position[] {
                position.NEXT,
                position.FINAL,
                position.IDLE
        };
        target_position_index = 0;
        intake_position = Intake_Position.IDLE;
        startPose = position.IDLE.getPose();

        start_time = clock.seconds();

    }
}
