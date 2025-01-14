package org.firstinspires.ftc.teamcode;


import static org.firstinspires.ftc.teamcode.DanDriveConstants.INIT_ANGLE;
import static org.firstinspires.ftc.teamcode.DanDriveConstants.Ka;
import static org.firstinspires.ftc.teamcode.DanDriveConstants.Kv;
import static org.firstinspires.ftc.teamcode.DanDriveConstants.TICKS_PER_INCH;
import static org.firstinspires.ftc.teamcode.DanDriveConstants.TICKS_PER_ROTATION;
import static org.firstinspires.ftc.teamcode.DanDriveConstants.trackwidth;
import static org.firstinspires.ftc.teamcode.DanDriveConstants.wheelbase;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.trajectories.DualLinearMotionProfile;
import org.firstinspires.ftc.teamcode.util.Pose2D;

import java.util.ArrayList;

@Config
@Autonomous(name="Samples Autonomous")
public class SamplesAutonomous extends LinearOpMode {

    /*
        TODO: Maybe wrap stuff like the arm angle, the drive train, the intakes, all in a separate class and call an update() method?
     */


    DcMotorEx fl, fr, bl, br, arm_rot, left_extend, right_extend;

    Servo intake_left, intake_right, intake_pitch, intake_roll;

    RevColorSensorV3 colorSensor;

    public double LATERAL_MULTIPLIER = 1.0;

    // PID Control constants to account for any error in position
    public static double MAX_FORWARD_GAIN = 0.4;
    public static double MAX_STRAFE_GAIN = 0.4;
    public static double MAX_ANG_GAIN = 0.3;

    public static double FORWARD_GAIN = 0.05, Xi = 0.001; // 30% power at 50 inches error
    public static double STRAFE_GAIN = 0.05, Yi = 0.001;
    public static double ANG_GAIN = 0.5/(Math.PI/2), Hi = 0.02;


    // this should be pretty self explanatory. Its called that so I can say "if we have a scoring element" lmao
    public boolean we_have_a_scoring_element = false;


    // These are intake positions. They assume that we have servo for pitch and a servo for roll. All numbers are on a scale of 0.0 to 1.0
    public enum intake_positions {
        IDLE(0.0, 0.0),
        BUCKET(1.0, 1.0),
        GROUND(0.0, 0.75),
        SPECIMEN(0.0, 0.5);

        public final double roll;
        public final double pitch;

        intake_positions(double roll, double pitch) {
            this.roll = roll;
            this.pitch = pitch;
        }
    }
    public intake_positions intake_position = intake_positions.IDLE;
    public double intake = 0.5; // This is the current power of the intake servos. 0.5 is not moving, 0.0 is one way, 1.0 is the other

    // Rotation positions, mostly just for a level of abstraction. Following that are a couple pid used things
    public enum rotations {
        IDLE(0),
        SAMPLES(65),
        SPECIMEN(75);

        public final double rotation;

        rotations(double rotation) {this.rotation = rotation;}
    }
    public rotations target_rotation = rotations.IDLE;
    public double min_rotation_power = 0.0;
    public double rotation_sum = 0;

    //Extension positions, in inches
    public enum extensions {
        IDLE(0),
        SAMPLES(47),
        SPECIMEN(25);

        public final double extension;

        extensions(double extension) {this.extension = extension;}

    }
    public extensions target_extension = extensions.IDLE;
    public double min_extension_power = 0.0;
    public double extension_sum = 0;

    // Positions the robot wants to be at
    public enum positions {
        START(new Pose2D(36, 72+trackwidth/2/2.54+1, 0)),
        BUCKET(new Pose2D(60, 60, Math.PI/4)),
        LEFT_SAMPLE(new Pose2D(63, 33, -Math.PI/4)),
        SUBMERSIBLE(new Pose2D(36, 12, -Math.PI)),
        RIGHT_SAMPLE(new Pose2D(48, 36, -Math.PI/2)),
        MIDDLE_SAMPLE(new Pose2D(60, 36, -Math.PI/2));

        public final Pose2D pose;

        positions(Pose2D pose) {this.pose = pose;}
    }
    // The order of positions the robot wants to move to
    public positions[] target_positions = {
            positions.BUCKET,
            positions.RIGHT_SAMPLE,
            positions.BUCKET,
            positions.MIDDLE_SAMPLE,
            positions.BUCKET,
            positions.LEFT_SAMPLE,
            positions.BUCKET,
            positions.SUBMERSIBLE
    };
    public int target_position_index = 0;
    public double x_sum = 0;
    public double y_sum = 0; // some pid control
    public double h_sum = 0;

    // This is used to store all the motion profiles that move from each position.
    DualLinearMotionProfile[] path;

    // Localizer
    public Localizer localizer;
    Pose2D poseEstimate;

    // Current state of the robot, IDLE is only used at the end of autonomous
    public enum states {
        IDLE,
        MOVING,
        GRABBING,
        DEPOSITING
    }
    public states state = states.MOVING;

    // Global variables so they can be accessed across methods
    public double arm_angle;
    public double arm_extension;

    // Status values for each PID system.
    public boolean profile_done;
    public double rotate_error;
    public double extend_error;
    public double intake_error;


    @Override
    public void runOpMode() {

        // Multiple telemetry, honestly doesn't quite matter.
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        // Motors :D
        fl = hardwareMap.get(DcMotorEx.class, "fl");
        fr = hardwareMap.get(DcMotorEx.class, "fr");
        bl = hardwareMap.get(DcMotorEx.class, "bl");
        br = hardwareMap.get(DcMotorEx.class, "br");

        fl.setDirection(DcMotorSimple.Direction.FORWARD);
        fr.setDirection(DcMotorSimple.Direction.REVERSE);
        bl.setDirection(DcMotorSimple.Direction.FORWARD);
        br.setDirection(DcMotorSimple.Direction.REVERSE);

        fl.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        fl.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        fr.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        fr.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        bl.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        bl.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        br.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        br.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        // More motors
        arm_rot = hardwareMap.get(DcMotorEx.class, "m7");
        left_extend = hardwareMap.get(DcMotorEx.class, "m6");
        right_extend = hardwareMap.get(DcMotorEx.class, "m5");

        arm_rot.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        arm_rot.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        arm_rot.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        arm_rot.setDirection(DcMotorSimple.Direction.REVERSE);
        left_extend.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        left_extend.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        left_extend.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        left_extend.setDirection(DcMotorSimple.Direction.FORWARD);
        right_extend.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        right_extend.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        right_extend.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        right_extend.setDirection(DcMotorSimple.Direction.REVERSE);

        // Right now we only have an servo for pitch, hopefully we end up with one for roll as well.
        intake_pitch = hardwareMap.get(Servo.class, "intake_pitch");
//        intake_roll = hardwareMap.get(Servo.class, "intake_roll");

        intake_pitch.setDirection(Servo.Direction.REVERSE);
        intake_pitch.setPosition(0);
//        intake_roll.setDirection(Servo.Direction.FORWARD);
//        intake_roll.setPosition(0);

//         Color sensor will be added, the autonomous sorta relies on it to actually work. It's possible to do without, but not recommended.
        colorSensor = hardwareMap.get(RevColorSensorV3.class, "color");
        while (!colorSensor.initialize()) {
            telemetry.addLine("Starting color sensor");
            telemetry.update();
        }
        we_have_a_scoring_element = colorSensor.red() > 160 && colorSensor.green() > 160 && colorSensor.getDistance(DistanceUnit.CM) < 5;

        // Localizer
        localizer = new Localizer(hardwareMap, telemetry);

        // Here, we iterate through each position we want to move to,
        ArrayList<DualLinearMotionProfile> temp_path = new ArrayList<>();
        Pose2D current_pose = positions.START.pose;

        for (positions pos : target_positions) {
            Pose2D target_pose = pos.pose;
            temp_path.add(
                    new DualLinearMotionProfile(
                            current_pose,
                            target_pose,
                            telemetry
                    )
            );
            current_pose = target_pose;
        }

        path = temp_path.toArray(new DualLinearMotionProfile[0]);

        waitForStart();

        while (!isStopRequested()) {

            update_things();

            intake = 0.5;

            switch (state) {
                case IDLE:
                    update_things();
                    idle();
                case MOVING:
                    double poseDistance = target_positions[target_position_index].pose.dist(poseEstimate);

                    if (poseDistance < 5) {
                        switch (target_positions[target_position_index]) {
                            case BUCKET:
                                target_rotation = rotations.SAMPLES;
                            case LEFT_SAMPLE:
                            case MIDDLE_SAMPLE:
                            case RIGHT_SAMPLE:
                                setIntakePosition(intake_positions.GROUND);

                        }
                    }

                    if (profile_done) {
                        switch (target_positions[target_position_index]) {
                            case BUCKET:
                                state = states.DEPOSITING;
                                break;
                            case LEFT_SAMPLE:
                            case RIGHT_SAMPLE:
                            case MIDDLE_SAMPLE:
                                state = states.GRABBING;
                                break;
                            case SUBMERSIBLE:
                                state = states.IDLE;
                                break;
                            case START:
                                state = states.MOVING;
                        }
                    }
                    break;

                case GRABBING:

                    intake_position = intake_positions.GROUND;

//                    we_have_a_scoring_element = colorSensor.red() > 160 && colorSensor.green() > 160 && colorSensor.getDistance(DistanceUnit.CM) < 5;
                    if (!we_have_a_scoring_element) {
                        left_extend.setPower(0.1);
                        right_extend.setPower(0.1);
                        intake = 1;
                        we_have_a_scoring_element = checkForSample();
                    } else {
                        intake_position = intake_positions.IDLE;

                        state = states.MOVING;
                        target_position_index++;
                    }
                    break;

                case DEPOSITING:

                    // TODO: make this better

                    if (we_have_a_scoring_element) {
                        if (target_extension == extensions.SAMPLES) {
                            if (extend_error < 3) {
                                intake = 0.0; // might change
                                we_have_a_scoring_element = checkForSample();
                            } else if (extend_error < 10) {
                                setIntakePosition(intake_positions.BUCKET);
                            }
                        } else if (rotate_error < 3 && target_rotation == rotations.SAMPLES) {
                            target_extension = extensions.SAMPLES;
                        }
                    } else {
                        if (target_extension == extensions.SAMPLES) {
                            if (intake_position == intake_positions.BUCKET) {
                                intake_position = intake_positions.IDLE;
                            } else if (intake_error < 0.5) { //fill this later
                                target_extension = extensions.IDLE;
                            }
                        } else if (extend_error < 5) {
                            target_rotation = rotations.IDLE;

                            state = states.MOVING;
                            target_position_index++;
                        }
                    }

            }

        }

    }

    public void update_things() {

        intake_left.setPosition(intake);
        intake_right.setPosition(intake);

        localizer.update();

        poseEstimate = localizer.getPoseEstimate();
        telemetry.addData("x", poseEstimate.x);
        telemetry.addData("y", poseEstimate.y);
        telemetry.addData("heading", poseEstimate.h);

        arm_angle = Math.max(0, (arm_rot.getCurrentPosition() / TICKS_PER_ROTATION * 360)) + INIT_ANGLE;
        arm_extension = ((left_extend.getCurrentPosition() + right_extend.getCurrentPosition()) / 2.0 / TICKS_PER_INCH);

        profile_done = movePID(path[target_position_index]);
        rotate_error = rotatePID(arm_angle);
        if (state != states.GRABBING) extend_error = extendPID(arm_extension);

        telemetry.addData("Current Target Position", target_positions[target_position_index]);
        telemetry.addData("Current Target Extension", target_extension);
        telemetry.addData("Current Target Rotation", target_rotation);

        telemetry.update();

    }

    // Movement feedforward and PID control

    public boolean movePID(DualLinearMotionProfile profile) {
        // profile.get_time() returns a pose for position, a pose for velocity, and a pose for acceleration in an array of poses, in that order.
        Pose2D[] at_time = profile.get_time();

        Pose2D poseTarget = at_time[0];

        // PID control to account for error in where we are over time.

        double error_x = poseTarget.x - poseEstimate.x;
        double error_y = poseTarget.y - poseEstimate.y;
        double error_heading = poseTarget.h - poseEstimate.h;

        if ((2 * Math.PI - Math.abs(error_heading)) < Math.abs(error_heading)) {
            error_heading = -Math.copySign(2 * Math.PI - Math.abs(error_heading), error_heading);
        }

        if (Math.abs(error_x) < 0.1) x_sum = 0;
        if (Math.abs(error_y) < 0.1) y_sum = 0;
        if (Math.abs(error_heading) < Math.PI) h_sum = 0;

        x_sum += error_x;
        y_sum += error_y;
        h_sum += error_heading;

        // P gains
        double forward = Range.clip(error_x * FORWARD_GAIN, -MAX_FORWARD_GAIN, MAX_FORWARD_GAIN);
        double strafe = Range.clip(error_y * STRAFE_GAIN, -MAX_STRAFE_GAIN, MAX_STRAFE_GAIN);
        double turn = Range.clip(error_heading * ANG_GAIN, -MAX_ANG_GAIN, MAX_ANG_GAIN);

        // I gain
        forward += x_sum * Xi;
        strafe += y_sum * Yi;
        turn += h_sum * Hi;

        forward = forward * Math.cos(-poseEstimate.h) - strafe * Math.sin(-poseEstimate.h);
        strafe = forward * Math.sin(-poseEstimate.h) + strafe * Math.cos(-poseEstimate.h);

        strafe *= LATERAL_MULTIPLIER;


        // Feedforward control
        Pose2D vel = at_time[1];
        Pose2D accel = at_time[2];

        double x = vel.x * Kv + accel.x * Ka + forward;
        double y = vel.y * Kv + accel.y * Ka + strafe;
        double w = (vel.h * Kv + accel.h * Ka) * (trackwidth/2 + wheelbase/2) + turn;

        double pfl = x - y - w;
        double pfr = x + y + w;
        double pbl = x + y - w;
        double pbr = x - y + w;

        fl.setPower(pfl);
        fr.setPower(pfr);
        bl.setPower(pbl);
        br.setPower(pbr);

        telemetry.addData("Forward", forward);
        telemetry.addData("Strafe", strafe);
        telemetry.addData("Heading", turn);

        return profile.is_traj_done();

    }

    // Its also possible to make some combined PID/kinematic controller for both rotation and extension, as polar coordinates?

    public double rotatePID(double arm_angle) {

        // todo: put the pid from DanChassisDrive in here and make it work

        return target_rotation.rotation - arm_angle;
    }

    public double extendPID(double arm_extension) {

        // todo: make some extension pid code using Kl and the arm_angle

        return target_extension.extension - arm_extension;
    }



    public double[] mecanumDrive(double drivey, double drivex, double turn) {

        double pfl, pfr, pbl, pbr;

        double power = Math.hypot(drivex, drivey);
        double theta = Math.atan2(drivey, drivex);

        double sin = Math.sin(theta - Math.PI/4);
        double cos = Math.cos(theta - Math.PI/4);
        double max = Math.max(Math.abs(sin), Math.abs(cos));

        pfl = power * cos/max + turn;
        pfr = power * sin/max - turn;
        pbl = power * sin/max + turn;
        pbr = power * cos/max - turn;

        if ((power + Math.abs(turn)) > 1) {
            pfl /= power + Math.abs(turn);
            pfr /= power + Math.abs(turn);
            pbl /= power + Math.abs(turn);
            pbr /= power + Math.abs(turn);

        }

        // negative cause it was all backwards
        return new double[]{-pfl, -pfr, -pbl, -pbr};

    }

    public void setIntakePosition(intake_positions pos) {

        // TODO: Implementation for the intake

    }

    public boolean checkForSample() {

        double green_red = (double) colorSensor.green() / colorSensor.red();
        double green_blue = (double) colorSensor.green() / colorSensor.blue();

        return colorSensor.getDistance(DistanceUnit.CM) < 2.5 && Math.abs(green_red - 4.2) < .25 && Math.abs(green_blue - 1.2) < .25;

    }

}
