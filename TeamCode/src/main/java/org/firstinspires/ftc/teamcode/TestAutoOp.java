package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.util.Pose2D;

@Autonomous(name="Test Autonomous mode")
public class TestAutoOp extends AutonomousOpMode {

    public enum position implements Position {
        START(new Pose2D(-16.5, -12, -Math.PI/2)),
        BASKET(new Pose2D(-3, 3, -Math.PI/4)),
        SAMPLE1(new Pose2D(1.5, 0.5, 0)),
        SAMPLE2(new Pose2D(1.5, 10.5, 0)),
        SAMPLE3(new Pose2D(9, 8, Math.PI/4));

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
        OUT(31);

        public final double ext;

        extension(double ext) {
            this.ext = ext;
        }

        public double getExtension() {
            return ext;
        }
    }

    @Override
    public void mainLoop() {

        we_have_a_scoring_element = checkForSample();

        switch (state) {
            case IDLE:


                if (we_have_a_scoring_element) {
                    if (target_positions[target_position_index] != position.BASKET) {
                        target_position_index++;
                        state = states.MOVING;
                    } else {
                        state = states.DEPOSITING;
                    }
                } else {
                    if (target_positions[target_position_index] != position.BASKET) {
                        state = states.GRABBING;
                        intake_position = Intake_Position.GROUND;
                    } else {
                        target_position_index++;
                        state = states.MOVING;
                    }
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
                }
                break;


            case MOVING:


                if (profile_done) {
                    state = states.IDLE;
                    break;
                }

                double dist = target_positions[target_position_index].getPose().dist(poseEstimate);
                double heading_error = (target_positions[target_position_index].getPose().h - poseEstimate.h);

                if (Math.abs(heading_error) > Math.PI) {
                    heading_error = -Math.copySign(2 * Math.PI - Math.abs(heading_error), heading_error);
                }


                if (target_positions[target_position_index] != position.BASKET && dist < 5) {
                    if (intake_position != Intake_Position.GROUND) intake_position = Intake_Position.GROUND;
                    if (dist < 1 && heading_error < 0.1) {
                        state = states.IDLE;
                        break;
                    }
                } else if (target_rotation != rotation.UP2 && dist < 1) {
                    target_rotation = rotation.UP2;
                    if (heading_error < 0.05) {
                        state = states.IDLE;
                        break;
                    }
                }

                break;


            case DEPOSITING:


                if (we_have_a_scoring_element) {
                    if (extension_error < 2 || target_extension == extension.OUT) {
                        if (rotation_error < 10 && target_rotation == rotation.UP2) {
                            if (extension_error < 1 && target_extension == extension.OUT) {
                                intake = 1.0;
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
                    } else if (rotation_error < 22.5) {
                        target_position_index++;
                        state = states.MOVING;
                    }

                }
                break;

        }

    }

    @Override
    public void variable_init() {

        target_rotation = rotation.IDLE;
        target_extension = extension.IDLE;
        target_positions = new position[] {
                position.BASKET,
                position.SAMPLE1,
                position.BASKET,
                position.SAMPLE2,
                position.BASKET,
                position.SAMPLE3,
                position.BASKET
        };
        target_position_index = 0;
        intake_position = Intake_Position.INIT;
        startPose = position.START.getPose();

        state = states.MOVING;

    }
}
