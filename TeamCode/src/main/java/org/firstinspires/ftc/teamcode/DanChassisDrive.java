package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.teamcode.DanDriveConstants.BASE_MAX_EXTENSION;
import static org.firstinspires.ftc.teamcode.DanDriveConstants.INIT_ANGLE;
import static org.firstinspires.ftc.teamcode.DanDriveConstants.Kg;
import static org.firstinspires.ftc.teamcode.DanDriveConstants.Kgl;
import static org.firstinspires.ftc.teamcode.DanDriveConstants.Kl;
import static org.firstinspires.ftc.teamcode.DanDriveConstants.TICKS_PER_INCH;
import static org.firstinspires.ftc.teamcode.DanDriveConstants.TICKS_PER_ROTATION;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.hardware.rev.RevColorSensorV3;
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
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.util.Pose2D;

@Config
@TeleOp(name = "DanChassisDrive")
public class DanChassisDrive extends LinearOpMode {

    RevColorSensorV3 colorSensor;

    public static double arm_rot_p = 0.02;
    public static double arm_rot_i = 0.0;
    public static double arm_rot_d = 0.0;
    public double arm_angle = 0;

    PIDFCoefficients arm_ext_pidg = new PIDFCoefficients(1/2.0, 0.0, 0.0, Kl);

    DcMotorEx fl, fr, bl, br, arm_rot, left_extend, right_extend;

    Servo intake_left, intake_right, intake_pitch, intake_roll, arm_pitch;

    Localizer localizer;

    FtcDashboard dashboard;

    ElapsedTime timer;

    // Arm Rotation control
    double[] arm_positions = {INIT_ANGLE, 65, 90};
    int current_arm_position_index = 0;

    // Intake rotation control
    double intake_pitch_pos = 0.5;
    double arm_pitch_pos = 0.5;

    // PID variables
    double prev_error = 0;
    double Ki_sum = 0.0;

    // booleans to allow for pressing the button once does a thing once
    boolean arm_rot_input = false;

    double prev_time;

    public double angle_to_servo_position(double angle) {
        return ((angle - arm_angle) / 600) + 0.5;
    }

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
        arm_rot.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        arm_rot.setDirection(DcMotorSimple.Direction.REVERSE);
        left_extend.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        left_extend.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        left_extend.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        left_extend.setDirection(DcMotorSimple.Direction.FORWARD);
        right_extend.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        right_extend.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        right_extend.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        right_extend.setDirection(DcMotorSimple.Direction.REVERSE);

        // two pitch, one roll :D
        arm_pitch = hardwareMap.get(Servo.class, "arm_pitch");
        intake_pitch = hardwareMap.get(Servo.class, "intake_pitch");
        intake_roll = hardwareMap.get(Servo.class, "intake_roll");

        arm_pitch.scaleRange(0.225, 1-0.225);
        arm_pitch.setPosition(0);
        intake_pitch.setDirection(Servo.Direction.REVERSE);
        intake_pitch.scaleRange(0.225, 1-0.225);
        intake_pitch.setPosition(0);
        intake_roll.scaleRange(1/6.0, 5/6.0);
        intake_roll.setPosition(0);

        intake_left = hardwareMap.get(Servo.class, "left");
        intake_right = hardwareMap.get(Servo.class, "right");
        intake_left.setDirection(Servo.Direction.REVERSE);

        colorSensor = hardwareMap.get(RevColorSensorV3.class, "color");
        while (!colorSensor.initialize()) {
            telemetry.addLine("Starting Color Sensor");
            telemetry.update();
        }
        telemetry.addData("We have a sample", checkForSample());

        localizer = new Localizer(hardwareMap, telemetry, new Pose2D(0, 0, 0));

        telemetry.addLine("Ready");
        telemetry.update();
        waitForStart();

        timer = new ElapsedTime(ElapsedTime.Resolution.SECONDS);
        timer.reset();

        while (opModeIsActive()) {

            mecanumDrive();

            arm_angle = Math.max(0, (arm_rot.getCurrentPosition() / TICKS_PER_ROTATION * 360)) + INIT_ANGLE;

//            double MAX_EXTENSION = BASE_MAX_EXTENSION * (1 - Math.cos(arm_angle) * (1 - (58.0/BASE_MAX_EXTENSION))); // 1 - (cos - cos*(fraction)) = 1 - cos + cos*fraction
            double MAX_EXTENSION = current_arm_position_index == 0 ? 47/2.54 : BASE_MAX_EXTENSION;

            double arm_extension = ((left_extend.getCurrentPosition() + right_extend.getCurrentPosition()) / 2.0 / TICKS_PER_INCH);
            double arm_extension_percentage = arm_extension / MAX_EXTENSION;

            double min_extend_power = Math.max(0, arm_extension * Kl * Math.sin(arm_angle));

            telemetry.addData("DELTA TIME", timer.time() - prev_time);

            telemetry.addData("Target Arm Rotation Position", current_arm_position_index);
            telemetry.addData("Arm Extension Position", arm_extension);
            telemetry.addData("Arm Rotation Position", arm_angle);

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

            arm_rot.setPower(rotatePID(arm_angle, arm_extension_percentage));
            arm_rot_input = gamepad2.dpad_up || gamepad2.dpad_down;

            telemetry.addData("Rotation Current", arm_rot.getCurrent(CurrentUnit.AMPS));
            telemetry.addData("Rotation Power", arm_rot.getPower());

            telemetry.addData("Extension Current", (left_extend.getCurrent(CurrentUnit.AMPS) + right_extend.getCurrent(CurrentUnit.AMPS)) / 2.0);
            telemetry.addData("Extension Power", (left_extend.getPower() + right_extend.getPower()) / 2.0);
            telemetry.addData("Extension Passive Power", min_extend_power);

            telemetry.addData("We Have a Sample", checkForSample());

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

            intake_left.setPosition(intake);
            intake_right.setPosition(intake);

            if (gamepad2.circle) {
                arm_pitch_pos = angle_to_servo_position(-20);
                intake_pitch_pos = angle_to_servo_position(-20);
            } else if (gamepad2.triangle) {
                arm_pitch_pos = angle_to_servo_position(0);
                intake_pitch_pos = angle_to_servo_position(0);
            }

            if (gamepad2.square) {
                arm_pitch.setPosition(0.0);
            } else {
                arm_pitch.setPosition(current_in_position);
            }

            if (gamepad1.triangle) {
                resetArmMotors();
            }

            prev_time = timer.time();

        }

    }

    public boolean checkForSample() {

        double green = colorSensor.green();
        double red = colorSensor.red();
        double blue = colorSensor.blue();
        double dist = colorSensor.getDistance(DistanceUnit.CM);

        double green_red = green/red;
        double green_blue = green/blue;

        TelemetryPacket packet = new TelemetryPacket(false);

        packet.put("Green", green);
        packet.put("Blue", blue);
        packet.put("Red", red);
        packet.put("Dist", (dist < 2));
        packet.put("Green/Red", (Math.abs(green_red - 1.25) < .25));
        packet.put("Green/Blue", (Math.abs(green_blue - 3.9) < .25));
        boolean value = (dist < 2) && (Math.abs(green_blue - 3.9) < .25) && (Math.abs(green_red - 1.25) < .25);
        packet.put("sample total", value);

        FtcDashboard.getInstance().sendTelemetryPacket(packet);

        return value;

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

    public double rotatePID(double arm_angle, double arm_extension_percentage) {
        double error = arm_positions[current_arm_position_index] - arm_angle;
        Ki_sum += Math.abs(error) < 5 ? error : 0;

        double p = error * arm_rot_p * (1 + Math.sin(Math.toRadians(arm_angle + 90)) * 1.2);

        double i = Ki_sum * arm_rot_i;

        double d = (error - prev_error) * arm_rot_d;

        double g = Math.sin(Math.toRadians(arm_angle + 90)) * Kg;

        double min_rot_power = p + i + d + g;

        min_rot_power *= ((arm_extension_percentage * Kgl) + 1); // adjust for the length of the arm

        if (Math.abs(error) >= 5) {
            Ki_sum = 0;
            if (current_arm_position_index == 0) {
                min_rot_power = -0.05;
            }
        }

        if (gamepad1.square) {
            min_rot_power = -1;
        }

        return min_rot_power;
    };

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
