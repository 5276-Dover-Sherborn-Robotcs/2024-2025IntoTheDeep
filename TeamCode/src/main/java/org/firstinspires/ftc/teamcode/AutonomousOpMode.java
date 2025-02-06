package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.teamcode.DanDriveConstants.BASE_MAX_EXTENSION;
import static org.firstinspires.ftc.teamcode.DanDriveConstants.INIT_ANGLE;
import static org.firstinspires.ftc.teamcode.DanDriveConstants.Ka;
import static org.firstinspires.ftc.teamcode.DanDriveConstants.Kg;
import static org.firstinspires.ftc.teamcode.DanDriveConstants.Kgl;
import static org.firstinspires.ftc.teamcode.DanDriveConstants.Kl;
import static org.firstinspires.ftc.teamcode.DanDriveConstants.Kv;
import static org.firstinspires.ftc.teamcode.DanDriveConstants.LATERAL_MULTIPLIER;
import static org.firstinspires.ftc.teamcode.DanDriveConstants.TICKS_PER_INCH;
import static org.firstinspires.ftc.teamcode.DanDriveConstants.TICKS_PER_ROTATION;
import static org.firstinspires.ftc.teamcode.DanDriveConstants.trackwidth;
import static org.firstinspires.ftc.teamcode.DanDriveConstants.wheelbase;

import android.annotation.SuppressLint;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.util.NanoClock;
import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.PIDCoefficients;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.trajectories.DualLinearMotionProfile;
import org.firstinspires.ftc.teamcode.util.Pose2D;

import java.util.ArrayList;

@Config
public abstract class AutonomousOpMode extends OpMode {

    abstract public void mainLoop();

    DcMotorEx fl, fr, bl, br, arm_rot, left_extend, right_extend;

    Servo intake_left, intake_right, arm_pitch, intake_pitch, intake_roll;

    RevColorSensorV3 colorSensor;

    public static double FORWARD_GAIN = 0.1, Xd = 0.015, Xi = 0.01; // 30% power at 50 inches error
    public static double STRAFE_GAIN = 0.1, Yd = 0.015, Yi = 0.01;
    public static double ANG_GAIN = .5, Hd = 0.2, Hi = 0.015;

    public static double forward_weight = 0.75;
    public static double strafe_weight = 0.75;
    public static double turn_weight = 0.25;

    public double prev_error_x = 0, x_sum = 0;
    public double prev_error_y = 0, y_sum = 0; // some pid control
    public double prev_error_h = 0, h_sum = 0;

    // this should be pretty self explanatory. Its called that so I can say "if we have a scoring element" lmao
    public boolean we_have_a_scoring_element = false;

    // These are intake positions. They assume that we have our goofy 4 bar arm setup. Pitches are local (global is overly complex), and seperate.
    public enum Intake_Position {
        IDLE(0.0, 135, 135),
        BUCKET_FORWARD(1.0, -65.0, -45.0),
        GROUND(0.0, -5.0, 0.0),
        SPECIMEN_FORWARD(0.0, 0.0, 0.0);

        public final double roll;
        public final double arm_pitch;
        public final double intake_pitch;

        Intake_Position(double roll, double pitch1, double pitch2) {
            this.roll = roll;
            this.arm_pitch = pitch1;
            if (Math.abs(pitch2 - pitch1) > 135) {
                this.intake_pitch = Math.copySign(135, pitch2 - pitch1);
            } else {
                this.intake_pitch = pitch2;
            }
        }
    }
    public Intake_Position intake_position = null;
    public Intake_Position old_intake_position = Intake_Position.IDLE;
    public double intake = 0.5; // This is the current power of the intake servos. 0.5 is not moving, 0.0 is one way, 1.0 is the other

    // Rotation positions, mostly just for a level of abstraction. Following that are a couple pid used things
    public interface Rotation {
        double getRotation();
    }
    public Rotation target_rotation = null;
    public double min_rotation_power = 0.0;
    public double prev_rotation_error = 0, rotation_sum = 0;
    public static PIDCoefficients rotation_pidg = new PIDCoefficients(
            .7/45, 0.0, 0.0
    );

    //Extension positions, in inches
    public interface Extension {
        double getExtension();
    }
    public Extension target_extension = null;
    public double min_extension_power = 0.0;
    public double prev_extension_error = 0, extension_sum = 0;
    public static PIDCoefficients extension_pidg = new PIDCoefficients(
            0.6/5, 0.0, 0.0
    );

    // Positions the robot wants to be at
    public interface Position {
        Pose2D getPose();
    }
    // The order of positions the robot wants to be at
    public Position[] target_positions = null;
    public int target_position_index = 0;
    public int old_target_position_index = 0;

    // This is used to store all the motion profiles that move from each position.
    DualLinearMotionProfile[] path;

    // Localizer
    public Localizer localizer;
    Pose2D poseEstimate;
    public Pose2D startPose;

    // Current state of the robot, IDLE is only used at the end of autonomous
    public enum states {
        IDLE,
        MOVING,
        GRABBING,
        DEPOSITING
    }
    public states state = states.MOVING;

    // Global variables so they can be accessed across methods
    public double arm_angle = 0;
    public double arm_extension = 0;
    public double arm_extension_percentage = 0;

    // Status values for each PID system.
    public boolean profile_done;
    public double rotate_error;
    public double extend_error;
    public double intake_error;

    NanoClock clock = NanoClock.system();

    abstract public void variable_init();

    @Override
    public void init() {

        variable_init();

        assert (target_rotation != null);

        assert (target_extension != null);

        assert (target_positions != null);

        assert (intake_position != null);

        assert (startPose != null);

        // Multiple telemetry, honestly doesn't quite matter.
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        // Motors :D
        fl = hardwareMap.get(DcMotorEx.class, "fl");
        fr = hardwareMap.get(DcMotorEx.class, "fr");
        bl = hardwareMap.get(DcMotorEx.class, "bl");
        br = hardwareMap.get(DcMotorEx.class, "br");

        fl.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        fr.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        fl.setDirection(DcMotorSimple.Direction.REVERSE);
        fr.setDirection(DcMotorSimple.Direction.FORWARD);
        bl.setDirection(DcMotorSimple.Direction.REVERSE);
        br.setDirection(DcMotorSimple.Direction.FORWARD);

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

        // two pitch, one roll :D
        arm_pitch = hardwareMap.get(Servo.class, "arm_pitch");
        intake_pitch = hardwareMap.get(Servo.class, "intake_pitch");
        intake_roll = hardwareMap.get(Servo.class, "intake_roll");


        arm_pitch.setDirection(Servo.Direction.REVERSE);
        arm_pitch.scaleRange(.5-0.225, .5+0.225);
        arm_pitch.setPosition(intake_position.arm_pitch/135 + .5);
        intake_pitch.scaleRange(.5-0.225, .5+0.225);
        intake_pitch.setPosition(intake_position.intake_pitch/135 + .5);
        intake_roll.scaleRange(1/6.0 - .025, 5/6.0 + .025);
        intake_roll.setPosition(intake_position.roll);

        intake_left = hardwareMap.get(Servo.class, "left");
        intake_right = hardwareMap.get(Servo.class, "right");
        intake_left.setDirection(Servo.Direction.REVERSE);

        colorSensor = hardwareMap.get(RevColorSensorV3.class, "color");
        while (!colorSensor.initialize()) {
            telemetry.addLine("Starting color sensor");
            telemetry.update();
        }
        we_have_a_scoring_element = colorSensor.red() > 160 && colorSensor.green() > 160 && colorSensor.getDistance(DistanceUnit.CM) < 5;

        // Localizer
        localizer = new Localizer(hardwareMap, telemetry, startPose);
        localizer.setPoseEstimate(startPose);

        // Here, we iterate through each position we want to move to,
        ArrayList<DualLinearMotionProfile> temp_path = new ArrayList<>();
        Pose2D current_pose = startPose;

        for (Position pos : target_positions) {
            Pose2D target_pose = pos.getPose();
            temp_path.add(
                    new DualLinearMotionProfile(
                            current_pose,
                            target_pose,
                            telemetry
                    )
            );
            current_pose = target_pose;
        }

        path = temp_path.toArray(new DualLinearMotionProfile[target_positions.length]);

    }

    public double angle_to_servo_position(double angle) {
        return (angle / 135) / 2 + 0.5;
    }

    @SuppressLint("DefaultLocale")
    @Override
    public void init_loop() {

        telemetry.addLine("Ready");

        telemetry.update();

    }

    @Override
    public void start() {

        // start the first motion profile
        path[0].start();

    }

    @Override
    public void loop() {

        intake = 0.5;

        mainLoop();

        update_things();

    }

    public void update_things() {

        intake_left.setPosition(intake);
        intake_right.setPosition(intake);

        localizer.update();

        localizer.telemetrize();

        poseEstimate = localizer.getPoseEstimate();

        arm_angle = Math.max(0, (arm_rot.getCurrentPosition() / TICKS_PER_ROTATION * 360)) + INIT_ANGLE;
        arm_extension = ((left_extend.getCurrentPosition() + right_extend.getCurrentPosition()) / 2.0 / TICKS_PER_INCH);
        arm_extension_percentage = arm_extension / BASE_MAX_EXTENSION;

        // I forgot to call start (or end on the old one), so we do that here every time the target position changes
        if (old_target_position_index != target_position_index) {
            path[old_target_position_index].end();
            path[target_position_index].start();
            h_sum = 0;
            x_sum = 0;
            y_sum = 0;
            old_target_position_index = target_position_index;
        }

        if (old_intake_position != intake_position) {
            arm_pitch.setPosition(intake_position.arm_pitch/135 + 0.5);
            intake_pitch.setPosition(intake_position.intake_pitch/135 + 0.5);
            intake_roll.setPosition(intake_position.roll);
            old_intake_position = intake_position;
        }

        profile_done = movePID();
        rotate_error = rotatePID();
        if (state != states.GRABBING) extend_error = extendPID();
//        intake_error = intakePID(); To be worked on


        telemetry.addData("Current Target Position", target_positions[target_position_index]);
        telemetry.addData("Current Extension", arm_extension);
        telemetry.addData("Current Extension Error", extend_error);
        telemetry.addData("Current Target Extension", target_extension);
        telemetry.addData("Current Target Extension num", target_extension.getExtension());
        telemetry.addData("Current Rotation", arm_angle);
        telemetry.addData("Current Rotation Error", rotate_error);
        telemetry.addData("Current Target Rotation", target_rotation);
        telemetry.addData("Current Target Rotation num", target_rotation.getRotation());

        telemetry.update();

    }

    // Movement feedforward and PID control

    public boolean movePID() {
        // profile.get_time() returns a pose for position, a pose for velocity, and a pose for acceleration in an array of poses, in that order.
        Pose2D[] at_time = path[target_position_index].get_state_at_time();

        Pose2D poseTarget = at_time[0];

        // PID control to account for error in where we are over time.

        // Error calculation
        double error_x = poseTarget.x - poseEstimate.x;
        double error_y = poseTarget.y - poseEstimate.y;
        double error_h = poseTarget.h - poseEstimate.h;
        if ((2 * Math.PI - Math.abs(error_h)) < Math.abs(error_h)) {
            error_h = -Math.copySign(2 * Math.PI - Math.abs(error_h), error_h);
        }

        // P gains
        double forward = error_x * FORWARD_GAIN;
        double strafe = error_y * STRAFE_GAIN;
        double turn = error_h * ANG_GAIN;

        // I gains
        // fun fact: strafe, forward, and turn... ARE DIFFERENT THINGS!!!
        x_sum += error_x;
        y_sum += error_y;
        if (Math.hypot(error_x, error_y) < 0.2) {
            x_sum = 0;
            y_sum = 0;
        }
        forward += x_sum * Xi;
        strafe += y_sum * Yi;

        h_sum += error_h;
        if (Math.abs(error_h) < 0.05) h_sum = 0;
        turn += h_sum * Hi;

        // D gains
        forward += (error_x - prev_error_x) / localizer.d_time * Xd;
        strafe += (error_y - prev_error_y) / localizer.d_time * Yd;
        turn += (error_h - prev_error_h) / localizer.d_time * Hd;

        // Weigh it, such that turn does not take precedence
        double[] weighed_control = weighedPID(forward, strafe, turn);
        forward = weighed_control[0];
        strafe = weighed_control[1];
        turn = weighed_control[2];

        // Rotate it
        forward = forward * Math.cos(-poseEstimate.h) - strafe * Math.sin(-poseEstimate.h);
        strafe = forward * Math.sin(-poseEstimate.h) + strafe * Math.cos(-poseEstimate.h);

        // Feedforward control
        Pose2D vel = at_time[1];
        Pose2D accel = at_time[2];

        double x = vel.x * Kv + accel.x * Ka + forward;
        double y = vel.y * Kv + accel.y * Ka + strafe;
        y *= LATERAL_MULTIPLIER;
        double w = (vel.h * Kv + accel.h * Ka) * (trackwidth/2 + wheelbase/2) + turn;

        // sum it all up
        double pfl = x - y - w;
        double pfr = x + y + w;
        double pbl = x + y - w;
        double pbr = x - y + w;

        // aaaaaaaand action
        fl.setPower(pfl);
        fr.setPower(pfr);
        bl.setPower(pbl);
        br.setPower(pbr);

        prev_error_x = error_x;
        prev_error_y = error_y;
        prev_error_h = error_h;

        boolean we_are_close_enough = (Math.hypot(error_x, error_y) < 0.5) && (Math.abs(error_h) < 0.05);

        return path[target_position_index].is_traj_done() && we_are_close_enough;
    }

    public double[] weighedPID(double forward, double strafe, double turn) {

        // the ratio of turn_weight to forward/strafe_weight is the "if all are one, what is the output:D

        double forward_weighed = forward * forward_weight;
        double strafe_weighed = strafe * strafe_weight;
        double turn_weighed = turn * turn_weight;

        double target = Math.max(Math.max(Math.abs(forward), Math.abs(strafe)), Math.abs(turn));

        double x_mult = Math.abs(Math.min(target/forward_weighed, target/strafe_weighed));
        double h_mult = Math.abs(target/turn_weighed);

        double mult = Math.min(x_mult, h_mult);

        return new double[]{
                Math.copySign(forward_weighed * mult, forward),
                Math.copySign(strafe_weighed * mult, strafe),
                Math.copySign(turn_weighed * mult, turn)
        };

    }

    // Its also possible to make some combined PID/kinematic controller for both rotation and extension, as polar coordinates?

    public double rotatePID() {

        double error = target_rotation.getRotation() - arm_angle;
        rotation_sum += error;

        double cos = Math.cos(Math.toRadians(arm_angle));

        double p = error * rotation_pidg.p * (1 + cos * 1.2);

        double i = rotation_sum * rotation_pidg.i;

        double d = (error - prev_rotation_error) / localizer.d_time * rotation_pidg.d;

        double g = cos * Kg;

        min_rotation_power = p + i + d + g;

        min_rotation_power *= ((arm_extension_percentage * Kgl) + 1); // adjust for the length of the arm

        if (Math.abs(error) >= 5) {
            rotation_sum = 0;
        }

        if (target_rotation.getRotation() == 0) {
            if (Math.abs(error) >= 25) min_rotation_power = -0.2;
            else min_rotation_power = 0;
        }

        arm_rot.setPower(min_rotation_power);

        prev_rotation_error = error;

        return error;
    }

    public double extendPID() {

        // todo: make some extension pid code using Kl and the arm_angle

//        double rotation_slow = Math.min(1, Math.max(0, (1 - Math.log10(Math.abs(rotate_error)))));
        double rotation_slow = 1;

        double error = (target_extension.getExtension() - arm_extension);
        extension_sum += error * rotation_slow;

        double p = error * extension_pidg.p;

        double i = extension_sum * extension_pidg.i;

        double d = (error - prev_extension_error) / localizer.d_time * extension_pidg.d;

        double g = Math.max(0, arm_extension * Kl * Math.sin(arm_angle));
        if (rotate_error < 5 && target_rotation.getRotation() == 0) g = 0;

        min_extension_power = p + i + d + g;

        if (Math.abs(error) < .5) extension_sum = 0;

        if (error - prev_extension_error < .1 && target_extension.getExtension() == 0 && Math.abs(error) > 0.2 && Math.abs(error) < .5) {
            left_extend.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            right_extend.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            left_extend.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            right_extend.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        }

        // 1 - log base 100 of rotate error. the larger the rotation error is, the slower it extends.
        min_extension_power *= rotation_slow;

        left_extend.setPower(min_extension_power);
        right_extend.setPower(min_extension_power);

        prev_extension_error = error;

        return target_extension.getExtension() - arm_extension;
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

    public void intakeControl() {

        /*

        There does exist a maximum difference between the arm pitch and the intake pitch, about +- 135 degrees (NON SYMMETRICAL, FIND IT)

         */

        arm_pitch.setPosition(intake_position.arm_pitch/135 + .5);
        intake_pitch.setPosition(intake_position.intake_pitch/135 + .5);
        intake_roll.setPosition(intake_position.roll);

    }

    public boolean checkForSample() {

        double green = colorSensor.green();
        double red = colorSensor.red();
        double blue = colorSensor.blue();
        double dist = colorSensor.getDistance(DistanceUnit.CM);

        double green_red = green/red;
        double green_blue = green/blue;

        telemetry.addData("Green/Blue", green_blue);
        telemetry.addData("Green/Red", green_red);
        telemetry.addData("Distance", dist);

        return (dist < 2) && (Math.abs(green_blue - 3.8) < .3) && (Math.abs(green_red - 1.25) < .3);

    }


    // stole it from linear op mode lol
    public final void idle() {
        // Otherwise, yield back our thread scheduling quantum and give other threads at
        // our priority level a chance to run
        Thread.yield();
    }




}
