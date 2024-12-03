package org.firstinspires.ftc.teamcode.tuning;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;

import java.util.Arrays;

@Autonomous(name="Max Velocity Tuner")
public class MaxVelocityTuner extends LinearOpMode {

    double max_velocity = 0;
    double[] maxs = {0, 0, 0, 0};

    DcMotorEx fl, fr, bl, br;
    DcMotorEx[] motors;

    FtcDashboard dashboard = FtcDashboard.getInstance();

    ElapsedTime timer = new ElapsedTime();

    @Override
    public void runOpMode() throws InterruptedException {

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

        motors = new DcMotorEx[]{fl, fr, bl, br};

        telemetry.addData("Ready","");
        telemetry.update();

        waitForStart();

        timer.reset();

        while (opModeIsActive()) {

            TelemetryPacket packet = new TelemetryPacket(false);

            double time = timer.time();

            if (time <= 1) {
                fl.setPower(time);
                fr.setPower(time);
                bl.setPower(time);
                br.setPower(time);
            } else if ((time - 1) < 0.5) {
                fl.setPower(1);
                fr.setPower(1);
                bl.setPower(1);
                br.setPower(1);
            }

            for (int i = 0; i < 4; i++) {

                double vel = motors[i].getVelocity() / 28;
                maxs[i] = Math.max(vel, maxs[i]);
                packet.addLine(String.format("MAX %s: %s", i, maxs[i]));

            }

            if (time >= 4) {
                fl.setPower(0);
                fr.setPower(0);
                bl.setPower(0);
                br.setPower(0);
            }

            double average = Arrays.stream(maxs).sum() / 4;
            packet.put("Average Max Velocity", average);
            dashboard.sendTelemetryPacket(packet);
            telemetry.update();

        }

    }
}
