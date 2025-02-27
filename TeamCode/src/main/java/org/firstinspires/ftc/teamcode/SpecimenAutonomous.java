package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.util.Pose2D;

@Config
@Autonomous(name="Specimen Auto")
public class SpecimenAutonomous extends AutonomousOpMode {

    public enum position implements Position {
        START(new Pose2D(-16, 36, 0)),

        OBSERVATION(new Pose2D(-12, 12, -5*Math.PI/8)),

        // 2 inches away from the bar
        SPECIMEN_BAR1(new Pose2D(12, 52, 0)),
        SPECIMEN_BAR12(new Pose2D(14, 52, 0)),
        SPECIMEN_BAR2(new Pose2D(12, 48, 0)),
        SPECIMEN_BAR22(new Pose2D(14, 48, 0)),
        SPECIMEN_BAR3(new Pose2D(12, 44, 0)),
        SPECIMEN_BAR32(new Pose2D(14, 44, 0)),
        SPECIMEN_BAR4(new Pose2D(12, 40, 0)),
        SPECIMEN_BAR42(new Pose2D(14, 40, 0)),

        // All sample positions are the middle of the close side
        // Todo: Make them 45 degree angles
        SAMPLE1(new Pose2D(1.5, -0.5, 0)), // First sample is at (21.5, 0.5)
        SAMPLE2(new Pose2D(1.5, -10.5, 0)), // Second sample is at (21.5, 10.5)
        SAMPLE3(new Pose2D(1.5, -10.5, Math.atan2(10, 20))); // Third sample is at (21.5, 20.5)

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
        SPECIMEN(80),
        UP(90);

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
        OUT(8);

        public final double ext;

        extension(double ext) {
            this.ext = ext;
        }

        public double getExtension() {
            return ext;
        }
    }

    boolean extended = false;
    boolean specimen = false;

    @Override
    public void main_loop() {

        we_have_a_scoring_element = checkForSample();

        switch (state) {
            case IDLE:


                if (we_have_a_scoring_element) {

                    if (target_positions[target_position_index] != position.SPECIMEN_BAR1 ||
                            target_positions[target_position_index] != position.SPECIMEN_BAR2 ||
                            target_positions[target_position_index] != position.SPECIMEN_BAR3 ||
                            target_positions[target_position_index] != position.SPECIMEN_BAR4) {
                        if (specimen) {
                            state = states.GRABBING;
                            intake_position = Intake_Position.GROUND;
                        } else {
                            target_position_index++;
                            state = states.MOVING;
                        }
                    } else {
                        state = states.DEPOSITING;
                        extended = false;
                    }

                } else {
                    if (target_positions[target_position_index] != position.SPECIMEN_BAR1) {
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


                if (target_positions[target_position_index] != position.SPECIMEN_BAR1 && dist < 5) {
                    if (intake_position != Intake_Position.GROUND) intake_position = Intake_Position.GROUND;
                    if (dist < 3) resetMotors();
                    if (dist < 1 && heading_error < 0.1) {
                        state = states.IDLE;
                        break;
                    }
                } else if (target_rotation != rotation.UP && dist < 3) {
                    target_rotation = rotation.UP;
                    if (heading_error < 0.05) {
                        state = states.IDLE;
                        break;
                    }
                }

                break;


            case DEPOSITING:
                if (we_have_a_scoring_element) {
                    if (rotation_error < 5 && target_rotation == rotation.SPECIMEN) {
                        if (target_extension == extension.OUT || extended) {
                            if (extension_error < 1) {
                                target_extension = extension.IDLE;
                            } else if (extension_error < 10) {
                                intake_position = Intake_Position.SPECIMEN_FORWARD;
                            }
                        } else {
                            target_extension = extension.OUT;
                            extended = true;
                        }

                    } else {
                        target_rotation = rotation.SPECIMEN;
                    }
                } else {
                    if (intake_position == Intake_Position.SPECIMEN_FORWARD) {
                        intake_position = Intake_Position.IDLE;
                    }
                    if (target_rotation == rotation.SPECIMEN) {
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
                position.SPECIMEN_BAR1,
                position.SPECIMEN_BAR2,
                position.START
        };
        target_position_index = 0;
        intake_position = Intake_Position.ZERO;
        startPose = position.START.getPose();

        state = states.MOVING;

    }
}
