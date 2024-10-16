package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp(name="Single Motor Capped Acceleration Test")
public class singularCappedAccel extends LinearOpMode {

    DcMotorEx motor;

    ElapsedTime timer = new ElapsedTime();

    final double MAX_ACCEL = 6000 * 60;

    @Override
    public void runOpMode() throws InterruptedException {

        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        motor = hardwareMap.get(DcMotorEx.class, "m1");

        motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        motor.setDirection(DcMotorSimple.Direction.FORWARD);
        motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        waitForStart();

        timer.reset();

        double prev_time = 0, time = 0, delta_time = 0;
        double prev_position = 0, position = 0, delta_position = 0;
        double output_rpm = 0, output_power = 0;

        while (opModeIsActive()) {

            double target_power = -gamepad1.left_stick_y;
            double target_rpm = target_power * 6000;

            position = motor.getCurrentPosition();
            time = timer.time();

            delta_position = position - prev_position;
            delta_time = time - prev_time;

            double rpm = (delta_position / delta_time) / 28 * 60;

            double error_rpm = target_rpm - rpm;
            double sign = Math.signum(error_rpm);

            double accel_rpm = MAX_ACCEL * delta_time;

//            telemetry.addLine(String.format("target_power: %d\ntarget_rpm: %d\nrpm: %d\nerror_rpm: %d", target_power, target_rpm, rpm, error_rpm));
            telemetry.addData("target_power", target_power);
            telemetry.addData("target_rpm", target_rpm);
            telemetry.addData("rpm", rpm);
            telemetry.addData("error_rpm", error_rpm);
            telemetry.addData("sign", sign);
            telemetry.addData("accel_rpm", accel_rpm);
            telemetry.addData("time", time);
            telemetry.addData("delta_time", delta_time);

            if (Math.abs(error_rpm) < accel_rpm) {
                output_rpm = target_rpm;
            } else {
                output_rpm = rpm + sign * accel_rpm;
            }

            output_power = Math.min(Math.max((output_rpm / 6000), -1), 1);

            telemetry.addData("output_rpm", output_rpm);
            telemetry.addData("output_power", output_power);

            motor.setPower(output_power);

            prev_position = position;
            prev_time = time;

            telemetry.update();

        }

    }
}
