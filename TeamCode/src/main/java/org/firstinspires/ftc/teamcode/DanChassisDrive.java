package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.teamcode.DanDriveConstants.*;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.teamcode.util.Localizer;

import java.util.Arrays;

@TeleOp(name = "DanChassisDrive")
public class DanChassisDrive extends LinearOpMode {

    // Kp = .2 power at 45 degree (1/8th of a rotation) error

    double Kp = 4.0/(TICKS_PER_ROTATION), Ki = 0.0, Kd = 0.0;

    DcMotorEx fl, fr, bl, br, arm_rot, left_extend, right_extend;

    Servo left, right, in_rot;

    Localizer localizer;

    FtcDashboard dashboard;

    @Override
    public void runOpMode() {

        dashboard = FtcDashboard.getInstance();

        telemetry = new MultipleTelemetry(telemetry, dashboard.getTelemetry());

        fl = hardwareMap.get(DcMotorEx.class, "m1");
        fr = hardwareMap.get(DcMotorEx.class, "m2");
        bl = hardwareMap.get(DcMotorEx.class, "m3");
        br = hardwareMap.get(DcMotorEx.class, "m4");

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
        in_rot.setPosition(1.0);

        localizer = new Localizer(hardwareMap, telemetry);

        telemetry.addLine("Ready");
        telemetry.update();
        waitForStart();

        double[] arm_positions = Arrays.stream(new double[]{0, 90, 100}).map(x -> (x / 360 * TICKS_PER_ROTATION)).toArray();
        int current_arm_position = 0;


        double prev_error = 0;
        double Ki_sum = 0.0;
        boolean arm_rot_input = false;

        while (opModeIsActive()) {

            TelemetryPacket packet = new TelemetryPacket(false);

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

            double arm_extension = ((left_extend.getCurrentPosition() + right_extend.getCurrentPosition()) / 2.0 / TICKS_PER_CM) / MAX_EXTENSION;
//            double arm_extension = 0;
            double arm_angle = arm_rot.getCurrentPosition() / TICKS_PER_ROTATION * 360 + INIT_ANGLE;
            double min_extend_power = arm_extension * Kl;
//            double min_extend_power = 0.0;


            packet.put("Target Arm Rotation Position", current_arm_position);
            packet.put("Arm Extension Position", arm_extension);
            packet.put("Arm Rotation Position", arm_angle);

            double min_rot_power = 0.0;
            double error = arm_positions[current_arm_position] - arm_rot.getCurrentPosition();
            packet.put("Arm Rotation Error", error / TICKS_PER_ROTATION * 360);
//            double error = 0;
            if (current_arm_position != 0 || Math.abs(error) >= 5) {

                // PIDG CONTROLLER WOOOOOOOOOOOOOOOOOOOOOOOOOOOOOO

                double p = error * Kp;
                packet.put("P component", p);
                Ki_sum += error;
                double i = Ki_sum * Ki;
                double d = (error - prev_error) * Kd;
                double g = Math.sin(Math.toRadians(arm_angle + 90 - INIT_ANGLE)) * Kg;
                min_rot_power = g;
                min_rot_power *= ((arm_extension * Kgl) + 1); // adjust for the length of the arm
                prev_error = error;
            } else {
                Ki_sum = 0;
            }

            if (gamepad1.dpad_up) {
                if (!arm_rot_input) current_arm_position = Math.min(current_arm_position + 1, 2);
                arm_rot_input = true;
//                telemetry.addLine("dpad up");
                arm_rot.setPower(1);
            } else if (gamepad1.dpad_down) {
                if (!arm_rot_input) current_arm_position = Math.max(current_arm_position - 1, 0);
                arm_rot_input = true;
//                telemetry.addLine("dpad down");
                arm_rot.setPower(-1);
            } else {
                arm_rot_input = false;
                arm_rot.setPower(min_rot_power);
            }

            packet.put("Arm Rotation Current", arm_rot.getCurrent(CurrentUnit.AMPS));
            packet.put("Passive Rotation Power", min_rot_power);

            if (gamepad1.dpad_right && arm_extension <= 1) {
//                telemetry.addLine("dpad right");
                left_extend.setPower(1);
                right_extend.setPower(1);
            } else if (gamepad1.dpad_left) {
//                telemetry.addLine("dpad left");
                left_extend.setPower(-1);
                right_extend.setPower(-1);
            } else {
                left_extend.setPower(min_extend_power);
                right_extend.setPower(min_extend_power);
            }

            packet.put("Arm Extension Current (average)", (left_extend.getCurrent(CurrentUnit.AMPS) + right_extend.getCurrent(CurrentUnit.AMPS)) / 2.0);

            dashboard.sendTelemetryPacket(packet);

//            telemetry.update();

            double intake = (gamepad1.left_trigger - gamepad1.right_trigger) / 2 + 0.5;
            left.setPosition(intake);
            right.setPosition(intake);

            if (gamepad1.x) {
                in_rot.setPosition(0.27);
            } else if (gamepad1.b) {
                in_rot.setPosition(1.0);
            } else if (gamepad1.y) {
                in_rot.setPosition(0.5);
            }

        }

    }

}
