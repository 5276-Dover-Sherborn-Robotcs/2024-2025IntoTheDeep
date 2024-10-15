package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@TeleOp(name = "DanChassisDrive")
public class DanChassisDrive extends LinearOpMode {

    DcMotorEx fl, fr, bl, br;

//    DcMotorEx motor;

    @Override
    public void runOpMode() {

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

//        motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        telemetry.addLine("Ready");
        telemetry.update();
        waitForStart();

        while (opModeIsActive()) {

//            double linpower = gamepad1.right_trigger - gamepad1.left_trigger;
//
//            motor.setPower(linpower);

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
//            telemetry.addData("Linear Power", linpower);
            telemetry.update();

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

            fl.setPower(pfl);
            fr.setPower(pfr);
            bl.setPower(pbl);
            br.setPower(pbr);
        }


    }

}
