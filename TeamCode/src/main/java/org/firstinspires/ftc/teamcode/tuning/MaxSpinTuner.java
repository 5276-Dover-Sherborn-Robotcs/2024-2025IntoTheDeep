package org.firstinspires.ftc.teamcode.tuning;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.util.NanoClock;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

public class MaxSpinTuner extends LinearOpMode {

    DcMotorEx fl, fr, bl, br;

    IMU imu;
    double prev_heading = 0;
    double prev_vh = 0;

    double direction = 1;
    double max_vel = 0;
    double max_accel = 0;

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

        imu = hardwareMap.get(IMU.class, "imu");

        RevHubOrientationOnRobot orientation = new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.RIGHT,
                RevHubOrientationOnRobot.UsbFacingDirection.UP
        );

        imu.initialize(new IMU.Parameters(orientation));

        double accel_multiplier = 0.5;
        double time_to_accel = 1/accel_multiplier;

        waitForStart();

        start_time = clock.seconds();

        while (opModeIsActive()) {

            double time = clock.seconds() - start_time;
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

            fl.setPower(-power);
            fr.setPower(power);
            bl.setPower(-power);
            br.setPower(power);

            measure_acceleration(dt);

            prev_time = time;

        }

    }


    public void measure_acceleration(double dt) {

        double heading = (imu.getRobotYawPitchRollAngles().getYaw() + Math.PI*2) % (Math.PI*2);
        double dh = heading - prev_heading;
        if (Math.abs(dh) > Math.PI) {
            dh = -Math.copySign(Math.PI*2-Math.abs(dh), dh);
        }
        double vh = dh / dt;
        double ah = (vh-prev_vh)/dt*direction;

        max_vel = Math.max(max_vel, vh);
        max_accel = Math.max(max_accel, ah);

        telemetry.addData("Velocity", vh);
        telemetry.addData("Acceleration", ah);
        telemetry.addData("Max Velocity", max_vel);
        telemetry.addData("Max Acceleration", max_accel);
        telemetry.addData("Their Velocity", imu.getRobotAngularVelocity(AngleUnit.RADIANS).zRotationRate);
        telemetry.update();

        prev_heading = heading;
        prev_vh = vh;

    }
}
