package org.firstinspires.ftc.teamcode.tuning;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.util.NanoClock;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.teamcode.util.Encoder;

public class MaxAccelerationTuner extends LinearOpMode {

    DcMotorEx fl, fr, bl, br;

    Encoder x, y;
    double dvx = 0, dvy = 0;
    double old_vx = 0, old_vy = 0;

    double max_vel = 0;
    double max_accel = 0;
    double direction = 1;

    NanoClock clock = NanoClock.system();
    double start_time = 0;
    double prev_time = 0;

    @Override
    public void runOpMode() throws InterruptedException {

        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        fl = hardwareMap.get(DcMotorEx.class, "fl");
        fr = hardwareMap.get(DcMotorEx.class, "fr");
        bl = hardwareMap.get(DcMotorEx.class, "bl");
        br = hardwareMap.get(DcMotorEx.class, "br");

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

        x = new Encoder(fl, clock);
        y = new Encoder(fr, clock);

        x.setDirection(Encoder.Direction.REVERSE);
        y.setDirection(Encoder.Direction.REVERSE);

        double accel_multiplier = 0.5;
        double time_to_accel = 1/accel_multiplier;

        waitForStart();

        start_time = clock.seconds();

        while (opModeIsActive()) {

            time = clock.seconds() - start_time;
            double dt = time - prev_time;


            double power = 0;

            if (time <= time_to_accel) {

                power = time*accel_multiplier*direction;

            } else if (time <= time_to_accel + 0.5) {

                power = 1;

            } else if (time <= 2*time_to_accel + 0.5){

                power = 1-((time-time_to_accel-0.5)*accel_multiplier*direction);

            } else if (time <= 2*time_to_accel + 2.5){

                direction = -direction;
                start_time = clock.seconds();
                accel_multiplier += 0.1;

            }

            telemetry.addData("Acceleration Multiplier", accel_multiplier);
            telemetry.addData("Time to Accelerate", time_to_accel);

            fl.setPower(power);
            fr.setPower(power);
            bl.setPower(power);
            br.setPower(power);

            measure_acceleration(dt);

            prev_time = time;

        }

    }

    public void measure_acceleration(double dt) {

        double vx = x.getCorrectedVelocity();
        double vy = y.getCorrectedVelocity();

        dvx = vx - old_vx;
        dvy = vy - old_vy;

        double ax = dvx / dt;
        double ay = dvy / dt;

        double accel = Math.hypot(ax, ay)*direction;

        telemetry.addData("Velocity", Math.hypot(vx, vy));
        max_vel = Math.max(Math.hypot(vx, vy)*direction, max_vel);
        telemetry.addData("Acceleration", accel);
        max_accel = Math.max(accel, max_accel);
        telemetry.addData("Max Velocity", max_vel);
        telemetry.addData("Max Acceleration", max_accel);
        telemetry.update();

    }
}
