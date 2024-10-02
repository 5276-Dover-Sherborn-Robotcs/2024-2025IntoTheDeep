package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@TeleOp(name = "DanChassisDrive")
public class DanChassisDrive extends LinearOpMode {

    DcMotorEx fl, fr, bl, br;

    @Override
    public void runOpMode() {

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

        telemetry.addLine("Ready");
        telemetry.update();
        waitForStart();

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

            pfl = power * cos/max;
            pfr = power * sin/max;
            pbl = power * sin/max;
            pbr = power * cos/max;

            if ((power + Math.abs(turn)) > 1) {
                pfl /= power + turn;
                pfr /= power + turn;
                pbl /= power + turn;
                pbr /= power + turn;

            }

            fl.setPower(pfl);
            fr.setPower(pfr);
            bl.setPower(pbl);
            br.setPower(pbr);
        }


    }

}
