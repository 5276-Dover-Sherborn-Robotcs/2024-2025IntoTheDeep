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

    Localizer localizer;

    IMU imu;

    DcMotorEx[] motors;

    DcMotorEx fl, fr, bl, br;

    final double l = 15, b = 8;

    double target_x, target_y, target_heading;

    final double MAX_VELOC = 6000.0 / 60 * 96 * Math.PI;

    final double MAX_ACCEL = 4000 * 60;

    @Override
    public void runOpMode() throws InterruptedException {

        localizer = new Localizer(hardwareMap, telemetry);

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
            localizer.telemetrize();

            double error_x = target_x - localizer.getX();
            double sign_x = Math.signum(error_x);
            double error_y = target_y - localizer.getY();
            double sign_y = Math.signum(error_y);
            double error_heading = target_heading - localizer.getHeading();
            double sign_heading = Math.signum(error_heading);

            double[] current_motor_velocity = localizer.getMotorVelocities(Localizer.dataType.RADIANS);
            double[] current_robot_velocity = {0, 0, 0};

            // Math to convert wheel velocities to robot velocities and vice versa can be found at https://github.com/acmerobotics/road-runner/blob/master/doc/pdf/Mobile_Robot_Kinematics_for_FTC.pdf
            // Matrix math converted to equations to make evaluating and reading simpler

            current_robot_velocity[0] = (96.0 / 8) * Arrays.stream(current_motor_velocity).sum();
            current_motor_velocity[0] *= -1; current_motor_velocity[2] *= -1;
            current_robot_velocity[1] = (96.0 / 8) * Arrays.stream(current_motor_velocity).sum();
            current_motor_velocity[1] *= -1; current_motor_velocity[2] *= -1;
            current_robot_velocity[2] = (1 / (l + b)) * Arrays.stream(current_motor_velocity).sum();

            double[] target_robot_velocity = {0, 0, 0};

            // TODO: implement motion profiling or some sort of control loop here

            double[] target_motor_velocity = {0, 0, 0, 0};
            target_motor_velocity[0] = (target_robot_velocity[0] - target_robot_velocity[1] - (l + b) * target_robot_velocity[2]) / 48;
            target_motor_velocity[1] = (target_robot_velocity[0] + target_robot_velocity[1] - (l + b) * target_robot_velocity[2]) / 48;
            target_motor_velocity[2] = (target_robot_velocity[0] - target_robot_velocity[1] + (l + b) * target_robot_velocity[2]) / 48;
            target_motor_velocity[3] = (target_robot_velocity[0] + target_robot_velocity[1] + (l + b) * target_robot_velocity[2]) / 48;

            for (int i = 0; i < 4; i++) {

                double target_power = target_motor_velocity[i] * 20 / 6000;

                motors[i].setPower(target_power);

            }

            telemetry.update();

        }

    }
}
