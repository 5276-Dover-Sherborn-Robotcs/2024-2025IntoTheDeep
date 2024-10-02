package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@TeleOp(name = "intaketesting")
public class intaketesting extends LinearOpMode {

    DcMotorEx motor;

    @Override
    public void runOpMode() {

        motor = hardwareMap.get(DcMotorEx.class, "m1");

        motor.setDirection(DcMotorSimple.Direction.FORWARD);
        motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        telemetry.addLine("Ready");
        telemetry.update();
        waitForStart();

        while (opModeIsActive()) {

            motor.setPower(gamepad1.left_stick_y);

        }


    }

}
