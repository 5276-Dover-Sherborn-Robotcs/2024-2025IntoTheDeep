package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.teamcode.DanDriveConstants.INIT_ANGLE;
import static org.firstinspires.ftc.teamcode.DanDriveConstants.Kg;
import static org.firstinspires.ftc.teamcode.DanDriveConstants.Kgl;
import static org.firstinspires.ftc.teamcode.DanDriveConstants.Kl;
import static org.firstinspires.ftc.teamcode.DanDriveConstants.TICKS_PER_INCH;
import static org.firstinspires.ftc.teamcode.DanDriveConstants.TICKS_PER_ROTATION;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.util.Pose2D;

@Config
@TeleOp(name = "DanChassisDrive")
public class DanChassisDrive extends LinearOpMode {

    RevColorSensorV3 colorSensor;

    public static double arm_rot_p = 0.015;
    public static double arm_rot_i = 0.0;
    public static double arm_rot_d = 0.0;
    public double arm_angle = 0;
    public double arm_extension = 0;
    public double arm_extension_percentage = 0;

    DcMotorEx fl, fr, bl, br, arm_rot, left_extend, right_extend, climbing_arm_motor;

    Servo intake_left, intake_right, intake_pitch, intake_roll, arm_pitch;

    Localizer localizer;

    FtcDashboard dashboard;

    ElapsedTime timer;

    // Arm Rotation control
    double[] arm_positions = {INIT_ANGLE, 75, 90};
    int current_arm_position_index = 0;

    // Intake rotation control
    public static double pitch_speed_control = 3; // tuned
    double intake_pitch_pos = 1.0;
    double arm_pitch_pos = 1.0;
    double intake_roll_pos = 0.0;

    // PID variables
    double prev_error = 0;
    double Ki_sum = 0.0;

    // booleans to allow for pressing the button once does a thing once
    boolean arm_rot_input = false;
    boolean flipped = false;
    boolean climbing_bar = false;
    boolean clamping = false, clamped = false;
    boolean we_have_a_sample = false;
    boolean specimen = false, g2_swapping = false;

    double climbing_bar_power = 0.0;

    double prev_time = 0;

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

        arm_pitch.setDirection(Servo.Direction.REVERSE);
        arm_pitch.scaleRange(.5-0.225, .5+0.225);
        arm_pitch.setPosition(1.0);
        intake_pitch.scaleRange(.5-0.225, .5+0.225);
        intake_pitch.setPosition(1.0);
        intake_roll.scaleRange(1/6.0 - .025, 5/6.0 + .025);
        intake_roll.setPosition(1.0);

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

            localizer.update();

            arm_angle = Math.max(0, (arm_rot.getCurrentPosition() / TICKS_PER_ROTATION * 360)) + INIT_ANGLE;

            double temp = (42-(19+9.5))/Math.cos(Math.toRadians(arm_angle));

            if (Math.abs(temp) > 42) temp = 42;

            telemetry.addData("Calculated Max Extension", temp);
            double MAX_EXTENSION = temp;

            arm_extension = ((left_extend.getCurrentPosition() + right_extend.getCurrentPosition()) / 2.0 / TICKS_PER_INCH);
            arm_extension_percentage = arm_extension / MAX_EXTENSION;

            double min_extend_power = Math.max(0, arm_extension * Kl * Math.sin(Math.toRadians(arm_angle)));


            telemetry.addData("DELTA TIME", timer.time() - prev_time);
            telemetry.addData("Target Arm Rotation Position", current_arm_position_index);
            telemetry.addData("Arm Extension Position", arm_extension);
            telemetry.addData("Arm Rotation Position", arm_angle);

            if (gamepad2.dpad_up) {
                if (!arm_rot_input) {
                    current_arm_position_index = Math.min(current_arm_position_index + 1, arm_positions.length - 1);
                    Ki_sum = 0;
                }
            } else if (gamepad2.dpad_down) {
                if (!arm_rot_input) {
                    current_arm_position_index = Math.max(current_arm_position_index - 1, 0);
                    Ki_sum = 0;
                }
            }

            arm_rot.setPower(rotatePID());
            arm_rot_input = gamepad2.dpad_up || gamepad2.dpad_down;

            we_have_a_sample = checkForSample();
            telemetry.addData("We Have a Sample", we_have_a_sample);

            telemetry.addData("X", localizer.getX());
            telemetry.addData("Y", localizer.getY());
            telemetry.addData("Heading", localizer.getHeading());

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

            if ((gamepad2.right_bumper || gamepad2.left_bumper) && !g2_swapping) {
                specimen = !specimen;
            }
            g2_swapping = gamepad2.right_bumper || gamepad2.left_bumper;

            double intake = (gamepad1.right_trigger - gamepad1.left_trigger) / 2.0 + 0.5;

            if (we_have_a_sample) intake = Math.max(0.5, intake);

            intake_left.setPosition(intake);
            intake_right.setPosition(intake);

            if (gamepad2.b && !flipped) {
                intake_roll_pos = 1 - intake_roll_pos;
                intake_roll.setPosition(intake_roll_pos);
            }
            flipped = gamepad2.b;


            double dt = localizer.d_time;

            arm_pitch_pos -= (gamepad2.left_stick_y / pitch_speed_control * dt);
            intake_pitch_pos -= (gamepad2.right_stick_y / pitch_speed_control * dt);

            if (Math.abs(intake_pitch_pos - arm_pitch_pos) > 0.4) {
                intake_pitch_pos = arm_pitch_pos + Math.copySign(0.4, intake_pitch_pos - arm_pitch_pos);
            }

            arm_pitch_pos = Math.min(1.0, Math.max(0, arm_pitch_pos));
            intake_pitch_pos = Math.min(1.0, Math.max(0, intake_pitch_pos));

            if (gamepad2.a) {
                arm_pitch_pos = -35./270 + .5;
                intake_pitch_pos = .5;
                intake_roll_pos = 0;
                intake_roll.setPosition(intake_roll_pos);
            } else if (gamepad2.y) {
                arm_pitch_pos = 1;
                intake_pitch_pos = 1;
            }else if (gamepad2.x) {
                if (specimen) {
                    arm_pitch_pos = -arm_angle/270 + .5;
                    intake_pitch_pos = .5;
                } else {
                    arm_pitch_pos = .4;
                    intake_pitch_pos = 0.0;
                }
                intake_roll_pos = 0;
                intake_roll.setPosition(intake_roll_pos);
            }

            if (current_arm_position_index == 0) {
                double y1 = Math.sin(Math.toRadians((arm_pitch_pos - 0.5) * 270)) * 4.72;
                double y2 = Math.sin(Math.toRadians((intake_pitch_pos - 0.5) * 270)) * 5.89;
                telemetry.addData("Y position of intake", y1 + y2);
                if (y1 + y2 < -3) {
                    if (y1 > -3) {
                        intake_pitch_pos = Math.asin((-3 - y1) / 5.89)/270 + .5;
                    } else {
                        arm_pitch_pos = Math.asin(-3 / 4.78)/270 + .5;
                        intake_pitch_pos = 0.5;
                    }
                }
            }

            telemetry.addData("Arm Pitch Pos", arm_pitch_pos);
            telemetry.addData("Arm Pitch Angle", (arm_pitch_pos - 0.5) * 270);
            telemetry.addData("Intake Pitch Pos", intake_pitch_pos);
            telemetry.addData("Intake Pitch Angle", (intake_pitch_pos - 0.5) * 270);

            telemetry.addData("Specimen Mode", specimen);

            arm_pitch.setPosition(arm_pitch_pos);
            intake_pitch.setPosition(intake_pitch_pos);

            prev_time = time;


            if (gamepad1.y) {
                resetArmMotors();
            }


            if (gamepad1.a && !climbing_bar) {
                climbing_bar_power = 0.2 - climbing_bar_power;
                // climbing_bar_motor.setPower(climbing_bar_power);
            }
            climbing_bar = gamepad1.a;


            if (gamepad1.x && !clamping) {
                clamped = !clamped;
            }
            clamping = gamepad1.x;


            prev_time = timer.time();

            telemetry.update();

        }

    }

    public boolean checkForSample() {

        double green = colorSensor.green();
        double red = colorSensor.red();
        double blue = colorSensor.blue();
        double dist = colorSensor.getDistance(DistanceUnit.CM);

        double green_red = green/red;
        double green_blue = green/blue;

        return (dist < 3);

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

    public double rotatePID() {
        double error = arm_positions[current_arm_position_index] - arm_angle;
        Ki_sum += Math.abs(error) < 5 ? error : 0;

        double p = error * arm_rot_p * (1 + Math.cos(Math.toRadians(arm_angle)) * 1.2);

        double i = Ki_sum * arm_rot_i;

        double d = (error - prev_error) * arm_rot_d;

        double g = Math.cos(Math.toRadians(arm_angle)) * Kg;

        double min_rot_power = p + i + d + g;

        min_rot_power *= ((arm_extension_percentage * Kgl) + 1); // adjust for the length of the arm

        if (Math.abs(error) >= 5) {
            Ki_sum = 0;
        }

        if (current_arm_position_index == 0) {
            min_rot_power = -0.05;
        }

        if (clamped) {
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
