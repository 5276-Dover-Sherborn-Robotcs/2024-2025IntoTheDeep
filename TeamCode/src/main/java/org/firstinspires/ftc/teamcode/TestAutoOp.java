package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.util.Pose2D;

@Autonomous(name="Test Autonomous mode")
public class TestAutoOp extends AutonomousOpMode {

    public enum position implements Position {
        IDLE(new Pose2D(0, 0, 0)),
        NEXT(new Pose2D(-12, 12, -Math.PI/4));

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
        UP(75.0),
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
        OUT(30);

        public final double ext;

        extension(double ext) {
            this.ext = ext;
        }

        public double getExtension() {
            return ext;
        }
    }



    double start_time = 0;

    boolean swapped_recently = false;
    boolean we_extended = false;
    boolean we_retracted = false;

    @Override
    public void mainLoop() {

        we_have_a_scoring_element = checkForSample();

        switch (state) {
            case IDLE:
                switch (target_position_index) {
                    case 2:
                        state = states.GRABBING;
                        target_position_index = 0;
                        break;
                    case 1:
                        state = states.DEPOSITING;
                        intake_position = Intake_Position.GROUND;
                        break;
                    case 0:
                        target_position_index++;
                        state = states.MOVING;
                        break;
                }

                break;
            case GRABBING:
                if (!we_have_a_scoring_element) {
                    left_extend.setPower(0.75);
                    right_extend.setPower(0.75);
                    intake = 0.0;
                } else {
                    state = states.IDLE;
                    intake_position = Intake_Position.IDLE;
                    start_time = clock.seconds();
                }
                break;
            case MOVING:
                if (profile_done) state = states.IDLE;
                else if (target_positions[target_position_index].getPose().dist(poseEstimate) < 5) {
                    if (target_position_index == 2) {
                        if (intake_position != Intake_Position.GROUND) intake_position = Intake_Position.GROUND;
                    }
                } else {
                    if (intake_position != Intake_Position.IDLE) intake_position = Intake_Position.IDLE;
                }
                break;
            case DEPOSITING:
                if (we_have_a_scoring_element) {
                    if (extension_error < 2 || target_extension == extension.OUT) {
                        if (rotation_error < 3 && target_rotation == rotation.UP2) {
                            if (extension_error < 2 && target_extension == extension.OUT) {
                                intake = .9;
                            }
                            target_extension = extension.OUT;
                            intake_position = Intake_Position.BUCKET_BACKWARD;

                        }
                        target_rotation = rotation.UP2;
                    }
                } else {
                    if (intake_position == Intake_Position.BUCKET_BACKWARD) {
                        intake_position = Intake_Position.IDLE;
                    }
                    if (target_rotation == rotation.UP2) {
                        if (target_extension == extension.OUT) {
                            target_extension = extension.IDLE;
                        } else if (extension_error < 3) {
                            target_rotation = rotation.IDLE;
                        }
                    } else if (rotation_error < 3) {
                        target_position_index++;
                        state = states.MOVING;
                    }

                }

        }

    }

    @Override
    public void variable_init() {

        target_rotation = rotation.IDLE;
        target_extension = extension.IDLE;
        target_positions = new position[] {
                position.IDLE,
                position.NEXT,
                position.IDLE
        };
        target_position_index = 0;
        intake_position = Intake_Position.GROUND;
        startPose = position.IDLE.getPose();

        state = states.GRABBING;

        start_time = clock.seconds();

    }
}
