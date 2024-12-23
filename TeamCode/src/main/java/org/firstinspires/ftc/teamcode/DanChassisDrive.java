package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.teamcode.DanDriveConstants.BASE_MAX_EXTENSION;
import static org.firstinspires.ftc.teamcode.DanDriveConstants.INIT_ANGLE;
import static org.firstinspires.ftc.teamcode.DanDriveConstants.Kg;
import static org.firstinspires.ftc.teamcode.DanDriveConstants.Kgl;
import static org.firstinspires.ftc.teamcode.DanDriveConstants.Kl;
import static org.firstinspires.ftc.teamcode.DanDriveConstants.TICKS_PER_CM;
import static org.firstinspires.ftc.teamcode.DanDriveConstants.TICKS_PER_ROTATION;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;

@TeleOp(name = "DanChassisDrive")
public class DanChassisDrive extends LinearOpMode {

    // Kp = .2 power at 45 degree (1/8th of a rotation) error

    double Kp = 1.4/90, Ki = 0.0001, Kd = 0.0;

    PIDFCoefficients arm_rot_pidg = new PIDFCoefficients(
            .7/45, 0.0, 0.0, Kg
    );

    DcMotorEx fl, fr, bl, br, arm_rot, left_extend, right_extend;

    Servo left, right, in_rot;

    Localizer localizer;

    FtcDashboard dashboard;

    ElapsedTime timer;

    @Override
    public void runOpMode() {

        dashboard = FtcDashboard.getInstance();

        telemetry = new MultipleTelemetry(telemetry, dashboard.getTelemetry());

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
        arm_rot.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        arm_rot.setDirection(DcMotorSimple.Direction.REVERSE);
        left_extend.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        left_extend.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        left_extend.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        left_extend.setDirection(DcMotorSimple.Direction.FORWARD);
        right_extend.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        right_extend.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        right_extend.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        right_extend.setDirection(DcMotorSimple.Direction.REVERSE);

        left = hardwareMap.get(Servo.class, "left");
        left.setDirection(Servo.Direction.FORWARD);
        right = hardwareMap.get(Servo.class, "right");
        right.setDirection(Servo.Direction.REVERSE);
        in_rot = hardwareMap.get(Servo.class, "in");
        in_rot.setDirection(Servo.Direction.FORWARD);
        in_rot.setPosition(0.9);

        localizer = new Localizer(hardwareMap, telemetry);

        telemetry.addLine("Ready");
        telemetry.update();
        waitForStart();

        // Arm Rotation control
        double[] arm_positions = {INIT_ANGLE, 65, 87};
        int current_arm_position_index = 0;

        // Intake rotation control
        double current_in_position = 1.0;

        // PID variables
        double prev_error = 0;
        double Ki_sum = 0.0;

        // booleans to allow for pressing the button once does a thing once
        boolean arm_rot_input = false;

        timer = new ElapsedTime(ElapsedTime.Resolution.SECONDS);
        timer.reset();
        double prev_time = 0;

        while (opModeIsActive()) {

            mecanumDrive();

            double arm_angle = Math.max(0, (arm_rot.getCurrentPosition() / TICKS_PER_ROTATION * 360)) + INIT_ANGLE;

//            double MAX_EXTENSION = BASE_MAX_EXTENSION * (1 - Math.cos(arm_angle) * (1 - (58.0/BASE_MAX_EXTENSION))); // 1 - (cos - cos*(fraction)) = 1 - cos + cos*fraction
            double MAX_EXTENSION = current_arm_position_index == 0 ? 47 : BASE_MAX_EXTENSION;

            double arm_extension = ((left_extend.getCurrentPosition() + right_extend.getCurrentPosition()) / 2.0 / TICKS_PER_CM);
            double arm_extension_percentage = arm_extension / MAX_EXTENSION;

            double min_extend_power = Math.max(0, arm_extension * Kl * Math.sin(arm_angle));

            telemetry.addData("DELTA TIME", timer.time() - prev_time);

            telemetry.addData("Target Arm Rotation Position", current_arm_position_index);
            telemetry.addData("Arm Extension Position", arm_extension);
            telemetry.addData("Arm Rotation Position", arm_angle);

            // PIDG-EONNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNN
            double error = arm_positions[current_arm_position_index] - arm_angle;
            Ki_sum += Math.abs(error) < 5 ? error : 0;

            double p = error * Kp * (1 + Math.sin(Math.toRadians(arm_angle + 90))*1.2);

            double i = Ki_sum * Ki;

            double d = (error - prev_error) * Kd;

            double g = Math.sin(Math.toRadians(arm_angle + 90)) * Kg;

            double min_rot_power = p + i + d + g;

            min_rot_power *= ((arm_extension_percentage * Kgl) + 1); // adjust for the length of the arm

            prev_error = error;

            if (Math.abs(error) >= 5) {
                Ki_sum = 0;
                if (current_arm_position_index == 0) {
                    min_rot_power = 0;
                }
            }

            if (gamepad2.dpad_up) {
                if (!arm_rot_input) {
                    current_arm_position_index = Math.min(current_arm_position_index + 1, arm_positions.length - 1);
                    Ki_sum = 0;
                }
//
//                current_arm_rotation_power = rateLimiter(current_arm_rotation_power, 1, 0.25 * (timer.time() - prev_time));

            } else if (gamepad2.dpad_down) {
                if (!arm_rot_input) {
                    current_arm_position_index = Math.max(current_arm_position_index - 1, 0);
                    Ki_sum = 0;
                }
//
//                current_arm_rotation_power = rateLimiter(current_arm_rotation_power, -.5, 0.25 * (timer.time() - prev_time));
            }

            arm_rot.setPower(min_rot_power);
            arm_rot_input = gamepad2.dpad_up || gamepad2.dpad_down;

            telemetry.addData("Arm Rotation Current", arm_rot.getCurrent(CurrentUnit.AMPS));
            telemetry.addData("Passive Rotation Power", min_rot_power);

            telemetry.addData("Arm Extension Current (average)", (left_extend.getCurrent(CurrentUnit.AMPS) + right_extend.getCurrent(CurrentUnit.AMPS)) / 2.0);

            localizer.update();

            telemetry.addData("X", localizer.getX());
            telemetry.addData("Y", localizer.getY());
            telemetry.addData("Heading", localizer.getHeading());

            telemetry.update();

//            telemetry.update();

            double triggers_input = (gamepad2.right_trigger - gamepad2.left_trigger);

            if (arm_extension_percentage > 1) {
                left_extend.setPower(Math.min(0, (triggers_input + min_extend_power)));
                right_extend.setPower(Math.min(0, (triggers_input + min_extend_power)));
            } else if (current_arm_position_index == 2) {
                left_extend.setPower(triggers_input);
                right_extend.setPower(triggers_input);
            } else if (current_arm_position_index == 0) {
                triggers_input = (gamepad2.right_trigger/2 - gamepad2.left_trigger);
                left_extend.setPower(triggers_input + min_extend_power);
                right_extend.setPower(triggers_input + min_extend_power);
            } else {
                left_extend.setPower(triggers_input + min_extend_power);
                right_extend.setPower(triggers_input + min_extend_power);
            }

            double intake = (gamepad2.right_stick_y + gamepad2.left_stick_y) / 2.0 + 0.5;

            if (in_rot.getPosition() < 0.25) intake = 1.0;

            left.setPosition(intake);
            right.setPosition(intake);

            if (gamepad2.circle) {
                current_in_position = 1.0;
            } else if (gamepad2.triangle) {
                current_in_position = 0.5;
            }

            if (gamepad2.square) {
                in_rot.setPosition(0.0);
            } else {
                in_rot.setPosition(current_in_position);
            }

            if (gamepad1.cross) {
                resetArmMotors();
            }

            if (gamepad1.square) {
                arm_rot.setPower(-1.0);
            }

            prev_time = timer.time();

        }

    }

    public void resetArmMotors() {

        timer.reset();

        telemetry.addLine("Resetting Arm Rotation");
        telemetry.update();
        while (arm_rot.getCurrent(CurrentUnit.AMPS) <= 6) {
            if (timer.time() < 2.0) {
                arm_rot.setPower(Math.max(-0.5, -timer.time()));
            } else {
                break;
            }
        }

        arm_rot.setPower(0);
        arm_rot.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        telemetry.addLine("Arm Rotation Reset");

        timer.reset();
        telemetry.addLine("Resetting Arm Extension");
        telemetry.update();
        while ((left_extend.getCurrent(CurrentUnit.AMPS) + right_extend.getCurrent(CurrentUnit.AMPS)) / 2.0 <= 7) {
            if (timer.time() < 1.0) {
                left_extend.setPower(Math.max(-0.5, -timer.time()));
                right_extend.setPower(Math.max(-0.5, -timer.time()));
            } else {
                break;
            }
        }

        left_extend.setPower(0);
        right_extend.setPower(0);

        while (timer.time() < 1) {
            telemetry.addLine("Arm Reset");
            telemetry.update();
        }

        left_extend.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        right_extend.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        arm_rot.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        left_extend.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        right_extend.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        timer.reset();
    }

    public double rateLimiter(double current_speed, double target_speed, double max_accel) {

        double change = Range.clip(target_speed-current_speed, -max_accel, max_accel);
        return current_speed + change;

    }

    public double rotatePID() {
        return 0;
    }

    public void mecanumDrive() {

        double pfl, pfr, pbl, pbr;

        double drivex = gamepad1.left_stick_x;
        double drivey = -gamepad1.left_stick_y;

        double turn = gamepad1.right_stick_x;

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
