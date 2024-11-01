package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp(name = "DanChassisDrive")
public class DanChassisDrive extends LinearOpMode {

    DcMotorEx fl, fr, bl, br;

    // acceleration in rps/s
    static double MAX_ACCEL = 200;

    double ofl, ofr, obl, obr;

    double[] outputs = {ofl, ofr, obl, obr};

    double prev_time = 0;

//    DcMotorEx motor;

    ElapsedTime timer = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);



    DcMotorEx[] motors;

    @Override
    public void runOpMode() {

        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        fl = hardwareMap.get(DcMotorEx.class, "m1");
        fr = hardwareMap.get(DcMotorEx.class, "m2");
        bl = hardwareMap.get(DcMotorEx.class, "m3");
        br = hardwareMap.get(DcMotorEx.class, "m4");

//        motor = hardwareMap.get(DcMotorEx.class, "m5");

        fl.setDirection(DcMotorSimple.Direction.FORWARD);
        fr.setDirection(DcMotorSimple.Direction.REVERSE);
        bl.setDirection(DcMotorSimple.Direction.FORWARD);
        br.setDirection(DcMotorSimple.Direction.REVERSE);

//        motor.setDirection(DcMotorSimple.Direction.FORWARD);

        fl.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        fl.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        fr.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        fr.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        bl.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        bl.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        br.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        br.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        motors = new DcMotorEx[]{fl, fr, bl, br};

//        motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        telemetry.addLine("Ready");
        telemetry.update();
        waitForStart();

        timer.reset();
        prev_time = 0;

        while (opModeIsActive()) {

            double pfl, pfr, pbl, pbr;

            double drivex = gamepad1.left_stick_x;
            double drivey = -gamepad1.left_stick_y;

            double turn = gamepad1.right_stick_x;

            double power = Math.hypot(drivex, drivey);
            double theta = Math.atan2(drivey, drivex);

            double sin = Math.sin(theta - Math.PI/4);
            double cos = Math.cos(theta - Math.PI/4);
            double max = Math.max(Math.abs(sin), Math.abs(cos));

            telemetry.addLine(String.format("drivex: %.3f || drivey: %.3f || turn: %.3f", drivex, drivey, turn));
            telemetry.addLine(String.format("power: %.3f || theta: %.3f || max: %.3f", power, theta, max));

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


            double[] target_powers = {pfl, pfr, pbl, pbr};
            double max_power = 0;
            for (double p : target_powers) {
                if (Math.abs(p) >= max_power) {
                    max_power = Math.abs(p);
                }
            }

            telemetry.addData("Max Power", max_power);

            double delta_time = timer.time() / 1000 - prev_time;
            prev_time += delta_time;

            for (int i = 0; i < 4; i++) {
                double accel = MAX_ACCEL * delta_time * ((max_power != 0) ? Math.abs(target_powers[i] / max_power) : 1);

                double difference = target_powers[i] * 100 - outputs[i];

                outputs[i] += Math.max(Math.min(difference, accel), -accel);
                motors[i].setPower(outputs[i] / 100);

                telemetry.addData(String.format("Output %s:", i), outputs[i]);
                telemetry.addData(String.format("Target Power %s:", i), target_powers[i]);

            }
            telemetry.update();

        }

    }

}
