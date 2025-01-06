package org.firstinspires.ftc.teamcode;


import static org.firstinspires.ftc.teamcode.DanDriveConstants.INIT_ANGLE;
import static org.firstinspires.ftc.teamcode.DanDriveConstants.Kg;
import static org.firstinspires.ftc.teamcode.DanDriveConstants.TICKS_PER_ROTATION;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;

@Config
@Autonomous(name="Blue Side Samples")
public class BlueSideSamples extends LinearOpMode {

    DcMotorEx fl, fr, bl, br, arm_rot, left_extend, right_extend;

    enum COMMAND {
        MOVE,
        ROTATE,
        EXTEND,
        NOTHING
    }

    SampleMecanumDrive drive;

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

    public double[] target_rotations = {
            0, 65, 0
    };
    public int target_rotation_index = 0;
    public double min_rotation_power = 0.0;
    public double rotation_sum = 0;

    public double[] target_extensions = {
            0, 1, 0
    };
    public int target_extension_index = 0;
    public double min_extension_power = 0.0;
    public double extension_sum = 0;

    public Pose2d[] target_positions = {
            new Pose2d(0, 0, 0),
            new Pose2d(12, 0, 0),
            new Pose2d(12, 12, 0),
            new Pose2d(0, 12, 0),
            new Pose2d(0, 0, 0)
    };
    public int target_position_index = 0;
    public double x_sum = 0;
    public double y_sum = 0;
    public double h_sum = 0;

    public COMMAND current_command = COMMAND.NOTHING;
    public COMMAND[] commands = {
            COMMAND.NOTHING,
            COMMAND.MOVE,
            COMMAND.MOVE,
            COMMAND.MOVE,
            COMMAND.MOVE
    };
    public int command_index = 0;

    @Override
    public void runOpMode() {

        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        drive = new SampleMecanumDrive(hardwareMap);

        drive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        drive.setPoseEstimate(target_positions[0]);

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

        waitForStart();

        while (!isStopRequested()) {

            Pose2d poseEstimate = drive.getPoseEstimate();
            telemetry.addData("x", poseEstimate.getX());
            telemetry.addData("y", poseEstimate.getY());
            telemetry.addData("heading", poseEstimate.getHeading());

            double arm_angle = Math.max(0, (arm_rot.getCurrentPosition() / TICKS_PER_ROTATION * 360)) + INIT_ANGLE;

            min_rotation_power = Math.sin(Math.toRadians(arm_angle + 90)) * Kg;

            drive.update();

            telemetry.addData("Current Target Position Index", target_position_index);
            telemetry.addData("Current Target Extension Index", target_extension_index);
            telemetry.addData("Current Target Rotation Index", target_rotation_index);

            switch (current_command) {
                case NOTHING:
                    command_index++;

                    if (command_index == commands.length) return;

                    current_command = commands[command_index];
                    switch (current_command) {
                        case MOVE:
                            target_position_index++;
                            break;
                        case ROTATE:
                            target_rotation_index++;
                            break;
                        case EXTEND:
                            target_extension_index++;
                            break;
                    }
                    break;
                case MOVE:
                    if (movePID(poseEstimate)) {
                        x_sum = 0;
                        y_sum = 0;
                        h_sum = 0;
                        current_command = COMMAND.NOTHING;
                        drive.setWeightedDrivePower(
                                new Pose2d(
                                        0,
                                        0,
                                        0
                                )
                        );
                    }
                    break;
                case ROTATE:
                    if (rotatePID(arm_angle)) {
                        current_command = COMMAND.NOTHING;
                        rotation_sum = 0;
                    }
                    break;
                case EXTEND:
                    if (extendPID(0)) {
                        current_command = COMMAND.NOTHING;
                        extension_sum = 0;
                    }
                    break;
            }

            telemetry.addData("Count", count);
            telemetry.update();

        }

        while (!isStopRequested()) {

        }

    }

    public boolean movePID(Pose2d poseEstimate) {

        double error_x = target_positions[target_position_index].getX() - poseEstimate.getX();
        double error_y = target_positions[target_position_index].getY() - poseEstimate.getY();
        double error_heading = target_positions[target_position_index].getHeading() - poseEstimate.getHeading();

        if ((2*Math.PI - Math.abs(error_heading)) < Math.abs(error_heading)) {
            error_heading = 2*Math.PI - Math.abs(error_heading);
        }

        if (Math.abs(error_x) < 0.2 && Math.abs(error_y) < 0.2 && Math.abs(error_heading) < Math.PI/180) {
            count++;
        } else {
            count = 0;
        }

        if (count == 50) {
            count = 0;
            return true;
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

        forward = forward * Math.cos(-poseEstimate.getHeading()) - strafe * Math.sin(-poseEstimate.getHeading());
        strafe = forward * Math.sin(-poseEstimate.getHeading()) + strafe * Math.cos(-poseEstimate.getHeading());

        strafe *= LATERAL_MULTIPLIER;

        mecanumDrive(forward, -strafe, -turn);

        telemetry.addData("Forward", forward);
        telemetry.addData("Strafe", strafe);
        telemetry.addData("Hedaing", turn);

        return false;

    }

    public boolean rotatePID(double arm_angle) {
        return false;
    }

    public boolean extendPID(double arm_extension) {
        return false;
    }

    public void mecanumDrive(double drivey, double drivex, double turn) {

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
        fl.setPower(-pfl);
        fr.setPower(-pfr);
        bl.setPower(-pbl);
        br.setPower(-pbr);

    }

}
