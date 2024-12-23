package org.firstinspires.ftc.teamcode;


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

    DcMotorEx fl, fr, bl, br;

    enum COMMAND {
        MOVE,
        ROTATE,
        EXTEND,
        NOTHING
    }

    SampleMecanumDrive drive;

    public static double TEST_X = 36;
    public static double TEST_Y = 0;
    public static double TEST_H = 0;

    public static double MAX_FORWARD_SPEED = 0.3;
    public static double MAX_STRAFE_SPEED = 0.3;
    public static double MAX_ANG_SPEED = 0.3;

    public static double FORWARD_GAIN = 0.4/12; // 30% power at 50 inches error
    public static double STRAFE_GAIN = 0.4/12;
    public static double ANG_GAIN = 0.3/(Math.PI/2);

    public double[] target_rotations = {
            0, 65, 0
    };
    public int target_rotation_index = 0;
    public double min_rotation_power = 0.0;

    public double[] target_extensions = {
            0, 1, 0
    };
    public int target_extension_index = 0;

    public Pose2d[] target_positions = {
            new Pose2d(0, 0, 0),
            new Pose2d(TEST_X, TEST_Y, TEST_Y),
            new Pose2d(0, 0, 0)
    };
    public int target_position_index = 0;

    public COMMAND current_command = COMMAND.NOTHING;
    public COMMAND[] commands = {
            COMMAND.NOTHING,
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

        waitForStart();

        while (!isStopRequested()) {
            drive.update();

            telemetry.addData("Current Target Position Index", target_position_index);
            telemetry.addData("Current Target Extension Index", target_extension_index);
            telemetry.addData("Current Target Rotation Index", target_rotation_index);

            switch (current_command) {
                case NOTHING:
                    command_index++;

                    if (command_index == commands.length) continue;

                    current_command = commands[command_index];
                    switch (current_command) {
                        case MOVE:
                            target_position_index++;
                            continue;
                        case ROTATE:
                            target_rotation_index++;
                            continue;
                        case EXTEND:
                            target_extension_index++;
                            continue;
                    }
                    continue;
                case MOVE:
                    if (movePID()) {
                        current_command = COMMAND.NOTHING;
                        drive.setWeightedDrivePower(
                                new Pose2d(
                                        0,
                                        0,
                                        0
                                )
                        );
                    }
                    continue;
                case ROTATE:
                    if (rotatePID()) current_command = COMMAND.NOTHING;
                    continue;
                case EXTEND:
                    if (extendPID()) current_command = COMMAND.NOTHING;
                    continue;
            }

        }

    }

    public boolean movePID() {

        Pose2d poseEstimate = drive.getPoseEstimate();
        telemetry.addData("x", poseEstimate.getX());
        telemetry.addData("y", poseEstimate.getY());
        telemetry.addData("heading", poseEstimate.getHeading());

        double error_x = target_positions[target_position_index].getX() - poseEstimate.getX();
        double error_y = target_positions[target_position_index].getY() - poseEstimate.getY();
        double error_heading = target_positions[target_position_index].getHeading() - poseEstimate.getHeading();

        if ((2*Math.PI - Math.abs(error_heading)) < Math.abs(error_heading)) {
            error_heading = 2*Math.PI - Math.abs(error_heading);
        }

        if (Math.abs(error_x) < 0.2 && Math.abs(error_y) < 0.2 && Math.abs(error_heading) < Math.PI/180) return true;

        double forward = Range.clip(error_x * FORWARD_GAIN, -MAX_FORWARD_SPEED, MAX_FORWARD_SPEED);
        double strafe = Range.clip(error_y * STRAFE_GAIN, -MAX_STRAFE_SPEED, MAX_STRAFE_SPEED);
        double turn = Range.clip(error_heading * ANG_GAIN, -MAX_ANG_SPEED, MAX_ANG_SPEED);

        forward = forward * Math.cos(-poseEstimate.getHeading()) - strafe * Math.sin(-poseEstimate.getHeading());
        strafe = forward * Math.sin(-poseEstimate.getHeading()) + strafe * Math.cos(-poseEstimate.getHeading());

        mecanumDrive(forward, strafe, turn);

        telemetry.addData("Forward", forward);
        telemetry.addData("Strafe", strafe);
        telemetry.addData("Hedaing", turn);
        telemetry.update();

        return false;

    }

    public boolean rotatePID() {
        return false;
    }

    public boolean extendPID() {
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
