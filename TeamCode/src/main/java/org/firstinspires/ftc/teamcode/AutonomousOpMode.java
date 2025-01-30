package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.teamcode.DanDriveConstants.INIT_ANGLE;
import static org.firstinspires.ftc.teamcode.DanDriveConstants.Ka;
import static org.firstinspires.ftc.teamcode.DanDriveConstants.Kg;
import static org.firstinspires.ftc.teamcode.DanDriveConstants.Kgl;
import static org.firstinspires.ftc.teamcode.DanDriveConstants.Kl;
import static org.firstinspires.ftc.teamcode.DanDriveConstants.Kv;
import static org.firstinspires.ftc.teamcode.DanDriveConstants.LATERAL_MULTIPLIER;
import static org.firstinspires.ftc.teamcode.DanDriveConstants.SERVO_MULTIPLIER;
import static org.firstinspires.ftc.teamcode.DanDriveConstants.TICKS_PER_INCH;
import static org.firstinspires.ftc.teamcode.DanDriveConstants.TICKS_PER_ROTATION;
import static org.firstinspires.ftc.teamcode.DanDriveConstants.trackwidth;
import static org.firstinspires.ftc.teamcode.DanDriveConstants.wheelbase;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.util.NanoClock;
import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.trajectories.DualLinearMotionProfile;
import org.firstinspires.ftc.teamcode.util.Pose2D;

import java.util.ArrayList;

public abstract class AutonomousOpMode extends OpMode {

    abstract public void mainLoop();

    DcMotorEx fl, fr, bl, br, arm_rot, left_extend, right_extend;

    Servo intake_left, intake_right, arm_pitch, intake_pitch, intake_roll;

    RevColorSensorV3 colorSensor;

    public static double FORWARD_GAIN = 0.1, Xd = 0.013, Xi = 0.0001; // 30% power at 50 inches error
    public static double STRAFE_GAIN = 0.1, Yd = 0.013, Yi = 0;
    public static double ANG_GAIN = 1.2, Hd = 0.2, Hi = 0;

    public double prev_error_x = 0, x_sum = 0;
    public double prev_error_y = 0, y_sum = 0; // some pid control
    public double prev_error_h = 0, h_sum = 0;

    // this should be pretty self explanatory. Its called that so I can say "if we have a scoring element" lmao
    public boolean we_have_a_scoring_element = false;;

    // These are intake positions. They assume that we have our goofy 4 bar arm setup. Pitches are global, and seperate.
    public enum Intake_Position {
        IDLE(0.0, 135, 0),
        BUCKET_FORWARD(1.0, -45.0, -90.0),
        GROUND(0.0, -45.0, 0.0),
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
    public Intake_Position intake_position = Intake_Position.IDLE;
    public double intake = 0.5; // This is the current power of the intake servos. 0.5 is not moving, 0.0 is one way, 1.0 is the other

    // Rotation positions, mostly just for a level of abstraction. Following that are a couple pid used things
    public interface Rotation {
        public double rotation = 0.0;
    }
    public Rotation target_rotation = null;
    public double min_rotation_power = 0.0;
    public double prev_rotation_error = 0, rotation_sum = 0;
    PIDFCoefficients rotation_pidg = new PIDFCoefficients(
            .7/45, 0.0, 0.0, Kg
    );

    //Extension positions, in inches
    public interface Extension {
        public double extension = 0.0;
    }
    public Extension target_extension = null;
    public double min_extension_power = 0.0;
    public double extension_sum = 0;

    // Positions the robot wants to be at
    public interface Position {
        public Pose2D pose = null;
    }
    // The order of positions the robot wants to be at
    public Position[] target_positions = null;
    public int target_position_index = 0;

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
    public double arm_angle;
    public double arm_extension;

    // Status values for each PID system.
    public boolean profile_done;
    public double rotate_error;
    public double extend_error;
    public double intake_error;

    NanoClock clock = NanoClock.system();
    double path_init_time = 0;

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

        // Right now we only have an servo for pitch, hopefully we end up with one for roll as well. and maybe a 2nd pitch??
        intake_pitch = hardwareMap.get(Servo.class, "intake_pitch");
        arm_pitch = hardwareMap.get(Servo.class, "arm_pitch");
        intake_roll = hardwareMap.get(Servo.class, "intake_roll");

        intake_pitch.setDirection(Servo.Direction.REVERSE);
        intake_pitch.setPosition(0);
        intake_roll.setPosition(0);

        colorSensor = hardwareMap.get(RevColorSensorV3.class, "color");
        while (!colorSensor.initialize()) {
            telemetry.addLine("Starting color sensor");
            telemetry.update();
        }
        we_have_a_scoring_element = colorSensor.red() > 160 && colorSensor.green() > 160 && colorSensor.getDistance(DistanceUnit.CM) < 5;

        // Localizer
        localizer = new Localizer(hardwareMap, telemetry);
        localizer.setPoseEstimate(startPose);

        double t0 = clock.seconds();

        // Here, we iterate through each position we want to move to,
        ArrayList<DualLinearMotionProfile> temp_path = new ArrayList<>();
        Pose2D current_pose = startPose;

        for (Position pos : target_positions) {
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

        path_init_time = clock.seconds()-t0;

    }

    @Override
    public void init_loop() {

        telemetry.addData("Time to path init", path_init_time);
        telemetry.addLine("Ready");

        telemetry.update();

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

        poseEstimate = localizer.getPoseEstimate();
        telemetry.addData("x", poseEstimate.x);
        telemetry.addData("y", poseEstimate.y);
        telemetry.addData("heading", poseEstimate.h);

        arm_angle = Math.max(0, (arm_rot.getCurrentPosition() / TICKS_PER_ROTATION * 360)) + INIT_ANGLE;
        arm_extension = ((left_extend.getCurrentPosition() + right_extend.getCurrentPosition()) / 2.0 / TICKS_PER_INCH);

        profile_done = movePID(path[target_position_index]);
        rotate_error = rotatePID(arm_angle);
        if (state != states.GRABBING) extend_error = extendPID(arm_extension);
        intake_error = intakePID();

        telemetry.addData("Current Target Position", target_positions[target_position_index]);
        telemetry.addData("Current Target Extension", target_extension);
        telemetry.addData("Current Target Rotation", target_rotation);

        telemetry.update();

    }

    // Movement feedforward and PID control

    public boolean movePID(DualLinearMotionProfile profile) {
        // profile.get_time() returns a pose for position, a pose for velocity, and a pose for acceleration in an array of poses, in that order.
        Pose2D[] at_time = profile.get_state_at_time();

        Pose2D poseTarget = at_time[0];
        Pose2D poseEstimate = localizer.getPoseEstimate();

        // PID control to account for error in where we are over time.

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

        if (Math.abs(error_h) < 0.1) h_sum = 0;
        if (Math.abs(error_x) < 0.1) x_sum = 0;
        if (Math.abs(error_y) < 0.1) y_sum = 0;

        x_sum += error_x;
        if (x_sum * error_x < 0) x_sum = 0;
        forward += x_sum * Xi;
        y_sum += error_y;
        if (y_sum * error_y < 0) y_sum = 0;
        forward += y_sum * Yi;
        h_sum += error_h;
        if (h_sum * error_h < 0) h_sum = 0;
        forward += h_sum * Hi;

        // D gains
        forward += (error_x - prev_error_x) / localizer.d_time * Xd;
        strafe += (error_y - prev_error_y) / localizer.d_time * Yd;
        turn += (error_h - prev_error_h) / localizer.d_time * Hd;

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

        double pfl = x - y - w;
        double pfr = x + y + w;
        double pbl = x + y - w;
        double pbr = x - y + w;

        fl.setPower(pfl);
        fr.setPower(pfr);
        bl.setPower(pbl);
        br.setPower(pbr);

        prev_error_x = error_x;
        prev_error_y = error_y;
        prev_error_h = error_h;

        double total_power = Math.abs(x) + Math.abs(y) + Math.abs(w);

        return profile.is_traj_done() && total_power < 0.1;
    }

    // Its also possible to make some combined PID/kinematic controller for both rotation and extension, as polar coordinates?

    public double rotatePID(double arm_angle) {

        double error = target_rotation.rotation - arm_angle;
        rotation_sum += Math.abs(error) < 5 ? error : 0;

        double sin = Math.sin(Math.toRadians(arm_angle + 90));

        double p = error * rotation_pidg.p * (1 + sin *1.2);

        double i = rotation_sum * rotation_pidg.i;

        double d = (error - prev_rotation_error) * rotation_pidg.d;

        double g = sin * rotation_pidg.f;

        min_rotation_power = p + i + d + g;

        min_rotation_power *= ((arm_extension * Kgl) + 1); // adjust for the length of the arm

        if (Math.abs(error) >= 5) {
            rotation_sum = 0;
            if (target_rotation.rotation == 0) {
                min_rotation_power = 0;
            }
        }

        arm_rot.setPower(min_rotation_power);

        return error;
    }

    public double extendPID(double arm_extension) {

        // todo: make some extension pid code using Kl and the arm_angle

        min_extension_power = Math.max(0, arm_extension * Kl * Math.sin(arm_angle));

        left_extend.setPower(min_extension_power);
        right_extend.setPower(min_extension_power);

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

    public double intakePID() {

        /*

        There does exist a maximum difference between the arm pitch and the intake pitch, about +- 135 degrees

         */

        Intake_Position pos = intake_position;

        double arm_pitch_angle = arm_pitch.getPosition() * SERVO_MULTIPLIER;
        double intake_pitch_angle = intake_pitch.getPosition() * SERVO_MULTIPLIER;

        double arm_pitch_error = intake_position.arm_pitch - arm_pitch_angle;
        double intake_pitch_error = intake_position.intake_pitch - intake_pitch_angle;
        double roll_error = intake_position.roll - intake_roll.getPosition();

        return arm_pitch_error + intake_pitch_error + roll_error;

    }

    public boolean checkForSample() {

        double green_red = (double) colorSensor.green() / colorSensor.red();
        double green_blue = (double) colorSensor.green() / colorSensor.blue();

        return colorSensor.getDistance(DistanceUnit.CM) < 2.5 && Math.abs(green_red - 4.2) < .25 && Math.abs(green_blue - 1.2) < .25;

    }


    // stole it from linear op mode lol
    public final void idle() {
        // Otherwise, yield back our thread scheduling quantum and give other threads at
        // our priority level a chance to run
        Thread.yield();
    }
}
