package org.firstinspires.ftc.teamcode.tuning;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;

import java.io.BufferedWriter;
import java.io.FileWriter;
import java.io.IOException;

public class MaxMotorRPM extends LinearOpMode {

    DcMotorEx fl, fr, bl, br;
    DcMotorEx[] motors;

    double ofl, ofr, obl, obr;
    double[] o = {ofl, ofr, obl, obr};

    double[] max_rpms = {0, 0, 0, 0};
    double MAX_RPM, prev_time = 0;

    ElapsedTime timer = new ElapsedTime(ElapsedTime.Resolution.SECONDS);

    @Override
    public void runOpMode() throws InterruptedException {

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

        telemetry.addLine("Ready");
        telemetry.update();
        waitForStart();

        timer.reset();



        while (opModeIsActive()) {

            double time = timer.time();
            if (time <= 1) {
                for (int i = 0; i < 4; i++) {
                    motors[i].setPower(time);
                    o[i] = motors[i].getCurrentPosition();
                }
            }
            if (time >= 5) {
                for (int i = 0; i < 4; i++) {
                    motors[i].setPower(0);
                }
                break;
            }
            if (time > 1) {
                for (int i = 0; i < 4; i++) {
                    motors[i].setPower(1);
                    max_rpms[i] = (motors[i].getCurrentPosition() - o[i]) / (time - 1);
                }
            }


        }

        String filepath = "constants.txt";

        try (BufferedWriter writer = new BufferedWriter(new FileWriter(filepath))) {
            writer.write(String.format("MAX RPM: %d", MAX_RPM));
        } catch (IOException e) {
            telemetry.addLine("smth went wrong writing the maxrpm frowny face");
        }

    }
}
