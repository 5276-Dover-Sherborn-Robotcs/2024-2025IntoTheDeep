package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name = "intaketesting")
public class intaketesting extends LinearOpMode {

    Servo l, r;

    @Override
    public void runOpMode() {

        l = hardwareMap.get(Servo.class, "l");
        r = hardwareMap.get(Servo.class, "r");

        r.setDirection(Servo.Direction.FORWARD);
        l.setDirection(Servo.Direction.REVERSE);

        telemetry.addLine("Ready");
        telemetry.update();
        waitForStart();

        while (opModeIsActive()) {

            double power = -gamepad1.left_stick_y / 2.0 + 0.5;

            l.setPosition(power);
            r.setPosition(power);

            telemetry.update();

        }


    }

}
