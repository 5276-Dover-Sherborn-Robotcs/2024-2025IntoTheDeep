package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

import java.util.Arrays;

@TeleOp(name="Localizer Test")
public class localizerTest extends LinearOpMode {

    Localizer localizer = new Localizer(hardwareMap, telemetry);

    IMU imu;

    DcMotorEx[] motors = localizer.getMotors();

    DcMotorEx fl, fr, bl, br;

    final double MAX_ACCEL = 100;

    @Override
    public void runOpMode() throws InterruptedException {

        motors = localizer.getMotors();

        imu = hardwareMap.get(IMU.class, "imu");

        RevHubOrientationOnRobot.LogoFacingDirection logoDirection = RevHubOrientationOnRobot.LogoFacingDirection.UP;
        RevHubOrientationOnRobot.UsbFacingDirection  usbDirection  = RevHubOrientationOnRobot.UsbFacingDirection.BACKWARD;

        RevHubOrientationOnRobot orientationOnRobot = new RevHubOrientationOnRobot(logoDirection, usbDirection);

        // Now initialize the IMU with this mounting orientation
        // Note: if you choose two conflicting directions, this initialization will cause a code exception.
        imu.initialize(new IMU.Parameters(orientationOnRobot));

        fl = motors[0];
        fr = motors[1];
        bl = motors[2];
        br = motors[3];

        while (opModeInInit()) {
            double imuheading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);
            if (imuheading < 0) imuheading += 2*Math.PI;
            telemetry.addLine("Localizer ready");
            localizer.telemetrize();
            telemetry.addData("IMU HEADING", imuheading);
            telemetry.update();
        }

        while (opModeIsActive()) {
            localizer.update();
            //PID can be done with rotations at the motor rather than velocity, however there will be an insane amount of noise

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
            localizer.telemetrize();

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

            double[] target_powers = new double[]{pfl, pfr, pbl, pbr};
            double[] target_rpms = Arrays.stream(target_powers).map(x -> x*6000).toArray();
            double[] rpms = localizer.getMotorVelocities(Localizer.dataType.MOTOR_RPM);

            for (int i = 0; i < 4; i++) {

                double output_power = rpms[i]/6000;

                double error = (target_rpms[i] - rpms[i]);
                if (error < (MAX_ACCEL * localizer.d_time)) {
                    output_power = target_powers[i];
                } else {
                    output_power += (MAX_ACCEL * localizer.d_time) / 6000;
                }

                motors[i].setPower(output_power);

            }

            telemetry.update();

        }

    }
}
