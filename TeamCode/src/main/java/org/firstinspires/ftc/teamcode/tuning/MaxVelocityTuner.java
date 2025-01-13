package org.firstinspires.ftc.teamcode.tuning;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.util.NanoClock;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.teamcode.util.Encoder;

@Config
@Autonomous(name="Max Velocity Tuner")
public class MaxVelocityTuner extends LinearOpMode {

    static double INCHES_PER_TICK = (4.8 / 2.54 * Math.PI) / 2000;
    public static double accel_multiplier = 1;

    double max_velocity = 0;
    double[] positions = {0, 0, 0, 0};

    DcMotorEx fl, fr, bl, br;
    DcMotorEx[] motors;
    Encoder x, y;

    FtcDashboard dashboard = FtcDashboard.getInstance();

    NanoClock clock = NanoClock.system();
    double start_time = 0;
    double prev_time = 0;

    @Override
    public void runOpMode() throws InterruptedException {

        telemetry = new MultipleTelemetry(telemetry, dashboard.getTelemetry());

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

        x = new Encoder(fl);
        x.setDirection(Encoder.Direction.REVERSE);
        y = new Encoder(fr);
        y.setDirection(Encoder.Direction.REVERSE);

        motors = new DcMotorEx[]{fl, fr, bl, br};

        telemetry.addData("Ready","");
        update_average();

        waitForStart();

        start_time = clock.seconds();

        while (opModeIsActive()) {

            double time = clock.seconds() - start_time;
            double dt = time - prev_time;
            prev_time = time;

            if (time <= 1/accel_multiplier) {
                fl.setPower(time*accel_multiplier);
                fr.setPower(time*accel_multiplier);
                bl.setPower(time*accel_multiplier);
                br.setPower(time*accel_multiplier);
            } else if (time < 1/accel_multiplier + 0.5) {
                fl.setPower(1);
                fr.setPower(1);
                bl.setPower(1);
                br.setPower(1);
            }

            telemetry.addData("Time", time);
            telemetry.addData("dT", dt);
            update_average();

            if (time >= 1/accel_multiplier + 2) {
                fl.setPower(0);
                fr.setPower(0);
                bl.setPower(0);
                br.setPower(0);
                break;
            }

        }

        while (opModeIsActive()) {
            idle();
        }

    }

    public void update_average() {

        double x_vel = x.getCorrectedVelocity()*INCHES_PER_TICK;
        double y_vel = y.getCorrectedVelocity()*INCHES_PER_TICK;
        double vel = Math.hypot(x_vel, y_vel);

        telemetry.addData("Their Corrected Velocity", vel);
        telemetry.addData("Their Raw Velocity", vel);
        telemetry.update();
    }

}
