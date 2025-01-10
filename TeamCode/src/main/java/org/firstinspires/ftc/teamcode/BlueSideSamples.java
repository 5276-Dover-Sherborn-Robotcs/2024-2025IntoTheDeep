package org.firstinspires.ftc.teamcode;


import static org.firstinspires.ftc.teamcode.DanDriveConstants.INIT_ANGLE;
import static org.firstinspires.ftc.teamcode.DanDriveConstants.Ka;
import static org.firstinspires.ftc.teamcode.DanDriveConstants.Kg;
import static org.firstinspires.ftc.teamcode.DanDriveConstants.Kv;
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
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.trajectories.DualLinearMotionProfile;
import org.firstinspires.ftc.teamcode.util.Pose2D;

import java.util.ArrayList;

@Config
@Autonomous(name="Blue Side Samples")
public class BlueSideSamples extends LinearOpMode {

    /*
        TODO: Maybe wrap stuff like the arm angle, the drive train, the intakes, all in a seperate class and call an update() method?
     */


    DcMotorEx fl, fr, bl, br, arm_rot, left_extend, right_extend;

    RevColorSensorV3 colorSensor;

    enum states {
        IDLE,
        MOVING,
        GRABBING,
        DEPOSITING
    }

    public int count = 0;

    public double LATERAL_MULTIPLIER = 1.1;

    public static double TEST_X = 36;
    public static double TEST_Y = 0;
    public static double TEST_H = 0;

    public static double MAX_FORWARD_SPEED = 0.4;
    public static double MAX_STRAFE_SPEED = 0.4;
    public static double MAX_ANG_SPEED = 0.3;

    public static double FORWARD_GAIN = 0.05, Xi = 0.001; // 30% power at 50 inches error
    public static double STRAFE_GAIN = 0.05, Yi = 0.001;
    public static double ANG_GAIN = 0.5/(Math.PI/2), Hi = 0.02;

    public boolean we_have_a_scoring_element = false;

    public enum rotations {
        IDLE,
        SAMPLES,
        SPECIMEN
    }
    public rotations target_rotation = rotations.IDLE;
    public double min_rotation_power = 0.0;
    public double rotation_sum = 0;

    public enum extensions {
        IDLE,
        SAMPLES,
        SPECIMEN
    }
    public extensions target_extension = extensions.IDLE;
    public double min_extension_power = 0.0;
    public double extension_sum = 0;

    public enum positions {
        START,
        BUCKET,
        LEFT_SAMPLE,
        MIDDLE_SAMPLE,
        RIGHT_SAMPLE,
        SUBMERSIBLE
    }
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
    public double y_sum = 0;
    public double h_sum = 0;

    DualLinearMotionProfile[] path;

    public Localizer localizer;

    @Override
    public void runOpMode() {

        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

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

        colorSensor = hardwareMap.get(RevColorSensorV3.class, "colorSensor");
        while (!colorSensor.initialize()) {
            telemetry.addLine("Starting color sensor");
            telemetry.update();
        }
        we_have_a_scoring_element = colorSensor.red() > 160 && colorSensor.green() > 160 && colorSensor.getDistance(DistanceUnit.CM) < 5;

        localizer = new Localizer(hardwareMap, telemetry);

        ArrayList<DualLinearMotionProfile> temp_path = new ArrayList<>();
        Pose2D current_pos = getTargetPosition(positions.START);

        for (positions pos : target_positions) {
            Pose2D target_pos = getTargetPosition(pos);
            temp_path.add(
                    new DualLinearMotionProfile(
                            current_pos,
                            target_pos,
                            telemetry
                    )
            );
            current_pos = target_pos;
        }

        path = temp_path.toArray(new DualLinearMotionProfile[0]);

        waitForStart();

        while (!isStopRequested()) {

            localizer.update();

            Pose2D poseEstimate = localizer.getPoseEstimate();
            telemetry.addData("x", poseEstimate.x);
            telemetry.addData("y", poseEstimate.y);
            telemetry.addData("heading", poseEstimate.h);

            double arm_angle = Math.max(0, (arm_rot.getCurrentPosition() / TICKS_PER_ROTATION * 360)) + INIT_ANGLE;
            double arm_extension = 0;

            boolean move = movePID(path[target_position_index]);
            double rotate = rotatePID(arm_angle);
            double extend = extendPID(arm_extension);

            telemetry.addData("Current Target Position", target_positions[target_position_index]);
            telemetry.addData("Current Target Extension", target_extension);
            telemetry.addData("Current Target Rotation", target_rotation);

            Pose2D poseTarget = getTargetPosition(target_positions[target_position_index]);
            double dist = poseTarget.dist(poseEstimate);

            if (target_positions[target_position_index] == positions.BUCKET) {
                if (dist < 5) target_rotation = rotations.SAMPLES;
            }


//                    switch (current_command) {
//                        case MOVE:
//                            target_position_index++;
//                            path[target_position_index].start();
//                            break;
//                        case ROTATE:
//                            if (target_positions[target_position_index] == positions.BUCKET) {
//                                target_rotation = rotations.SAMPLES;
//                            } else {
//                                target_rotation = rotations.IDLE;
//                            }
//                            break;
//                        case EXTEND:
//                            if (target_positions[target_position_index] == positions.BUCKET) {
//                                target_extension = extensions.SAMPLES;
//                            } else {
//                                target_extension = extensions.IDLE;
//                            }
//                            break;
//                    }
            if (movePID(path[target_position_index])) {

                if

            }

            if (getTargetPosition(positions.BUCKET).dist(localizer.getPoseEstimate()) < 5) {
                target_rotation = rotations.SAMPLES;
            }

            telemetry.addData("Count", count);
            telemetry.update();

        }

        while (!isStopRequested()) {

        }

    }

    public boolean movePID(DualLinearMotionProfile profile) {
        Pose2D[] at_time = profile.get_time();

        Pose2D poseTarget = at_time[0];
        Pose2D poseEstimate = localizer.getPoseEstimate();

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
        double forward = Range.clip(error_x * FORWARD_GAIN, -MAX_FORWARD_SPEED, MAX_FORWARD_SPEED);
        double strafe = Range.clip(error_y * STRAFE_GAIN, -MAX_STRAFE_SPEED, MAX_STRAFE_SPEED);
        double turn = Range.clip(error_heading * ANG_GAIN, -MAX_ANG_SPEED, MAX_ANG_SPEED);

        // I gain
        forward += x_sum * Xi;
        strafe += y_sum * Yi;
        turn += h_sum * Hi;

        forward = forward * Math.cos(-poseEstimate.h) - strafe * Math.sin(-poseEstimate.h);
        strafe = forward * Math.sin(-poseEstimate.h) + strafe * Math.cos(-poseEstimate.h);

        strafe *= LATERAL_MULTIPLIER;

        Pose2D vel = at_time[1];
        Pose2D accel = at_time[2];

        double x = vel.x * Kv + accel.x * Ka + forward;
        double y = vel.y * Kv + accel.y * Ka + strafe;
        double w = (vel.h * Kv + accel.h * Ka) * (trackwidth/2 + wheelbase/2) + turn;

        double pfl = x - y - w;
        double pfr = x + y + w;
        double pbl = x + y - w;
        double pbr = x - y + w;

        telemetry.addData("Forward", forward);
        telemetry.addData("Strafe", strafe);
        telemetry.addData("Heading", turn);

        return profile.is_traj_done();

    }

    public double rotatePID(double arm_angle) {

        // todo: put the pid from DanChassisDrive in here and make it work

        return getTargetRotation(target_rotation) - arm_angle;
    }

    public double extendPID(double arm_extension) {

        // todo: make some extension pid code using Kl and the arm_angle

        return getTargetExtension(target_extension) - arm_extension;
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


    /*
        Takes a position enum and returns a pose2d
    */
    public Pose2D getTargetPosition(positions pos) {
        switch (pos) {
            case START:
                return new Pose2D(36, 72+trackwidth/2/2.54+1, 0);
            case BUCKET:
                return new Pose2D(60, 60, Math.PI/4);
            case LEFT_SAMPLE:
                return new Pose2D(63, 33, -Math.PI/4);
            case SUBMERSIBLE:
                return new Pose2D(36, 12, -Math.PI);
            case RIGHT_SAMPLE:
                return new Pose2D(48, 36, -Math.PI/2);
            case MIDDLE_SAMPLE:
                return new Pose2D(60, 36, -Math.PI/2);
        }

        return new Pose2D(-1000, -1000, 0);
    }

    public double getTargetRotation(rotations rot) {
        switch (rot) {
            case IDLE:
                return 0.0;
            case SAMPLES:
                return 65.0;
            case SPECIMEN:
                return 70.0;
        }
        return 0.0;
    }

    public double getTargetExtension(extensions ext) {
        switch (ext) {
            case IDLE:
                return 0.0;
            case SPECIMEN:
                return 0.3;
            case SAMPLES:
                return 0.85;
        }
        return 0.0;
    }

}
