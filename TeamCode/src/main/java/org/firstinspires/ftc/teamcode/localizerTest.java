package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.teamcode.DriveConstants.Ka;
import static org.firstinspires.ftc.teamcode.DriveConstants.Kv;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode.trajectories.LinearMotionProfile;
import org.firstinspires.ftc.teamcode.trajectories.MotionProfile;

@TeleOp(name="Localizer Test")
public class localizerTest extends LinearOpMode {

    enum STATE {
        IDLE,
        RUNNING,
        DONE
    }

    private STATE state = STATE.IDLE;

    Localizer localizer;

    DcMotorEx[] motors;

    DcMotorEx fl, fr, bl, br;

    double[] outputs = {0, 0, 0, 0};

    final double l = 15, b = 8;

    MotionProfile motionProfile;

    @Override
    public void runOpMode() throws InterruptedException {

        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        localizer = new Localizer(hardwareMap, telemetry);

        IMU imu = localizer.imu;

        motors = localizer.getMotors();

        fl = motors[0];
        fr = motors[1];
        bl = motors[2];
        br = motors[3];

        while (opModeInInit()) {
            telemetry.addLine("Localizer ready");
            localizer.telemetrize();
            telemetry.update();
        }

        localizer.reset();
        boolean wasy = false;
        double timey = 0.0;

        while (opModeIsActive()) {
            localizer.telemetrize();
            localizer.update();

            switch (state) {
                case IDLE:

                    if (gamepad1.y) {
                        if (!wasy) {
                            timey = localizer.time();
                        }
                        if (wasy && timey != 0.0 && (timey - localizer.time()) >= 2) {
                            localizer.DEBUGGING = true;
                            timey = 0.0;
                        }
                    }

                    wasy = gamepad1.y;

                    double[] target_powers = get_mecanum_powers(gamepad1.left_stick_x, -gamepad1.left_stick_y);

                    for (int i = 0; i < 4; i++) telemetry.addData("TARGET POWER", "WHEEL %s: %.3f", i, target_powers[i]);

                    double max_power = 0;
                    for (double p : target_powers) {
                        if (Math.abs(p) >= max_power) {
                            max_power = Math.abs(p);
                        }
                    }

                    double delta_time = localizer.d_time;

                    for (int i = 0; i < 4; i++) {
                        double accel = delta_time * ((max_power != 0) ? Math.abs(target_powers[i] / max_power) : 1);

                        double difference = target_powers[i] - outputs[i];

                        outputs[i] += Math.max(Math.min(difference, accel), -accel);
                        motors[i].setPower(outputs[i]);
                    }

                    if (gamepad1.a) {
                        // todo: start a forward motion profiling

                        motionProfile = new LinearMotionProfile(new Pose2D(DistanceUnit.CM, 0, 0, AngleUnit.RADIANS, 0), new Pose2D(DistanceUnit.CM, 30, 0, AngleUnit.RADIANS, 0));
                        motionProfile.start();

                        state = STATE.RUNNING;
                    }

                    break;
                case RUNNING:

                    Pose2D vel = motionProfile.traj_vel_time();
                    Pose2D accel = motionProfile.traj_acc_time();

                    double x = Kv * vel.getX(DistanceUnit.CM) + Ka * accel.getX(DistanceUnit.CM);
                    double y = Kv * vel.getY(DistanceUnit.CM) + Ka * accel.getY(DistanceUnit.CM);
                    double[] outputs = {x, y, y, x};

                    for (int i = 0; i < 4; i++) motors[i].setPower(outputs[i]);

                    if (motionProfile.is_traj_done()) {
                        state = STATE.DONE;
                    }
                    break;
                case DONE:
                    for (int i = 0; i < 4; i++) motors[i].setPower(0);
                    motionProfile.end();
                    state = STATE.IDLE;
                    break;
            }

            telemetry.update();

//            double[] target_robot_velocity = {0, 0, 0};
//
//            // TODO: implement motion profiling or some sort of control loop here
//
//            double[] target_motor_velocity = {0, 0, 0, 0};
//            target_motor_velocity[0] = (target_robot_velocity[0] - target_robot_velocity[1] - (l + b) * target_robot_velocity[2]) / 48;
//            target_motor_velocity[1] = (target_robot_velocity[0] + target_robot_velocity[1] - (l + b) * target_robot_velocity[2]) / 48;
//            target_motor_velocity[2] = (target_robot_velocity[0] - target_robot_velocity[1] + (l + b) * target_robot_velocity[2]) / 48;
//            target_motor_velocity[3] = (target_robot_velocity[0] + target_robot_velocity[1] + (l + b) * target_robot_velocity[2]) / 48;
//
//            for (int i = 0; i < 4; i++) {
//
//                double target_power = target_motor_velocity[i] * 20 / 6000;
//
//                motors[i].setPower(target_power);
//
//            }

            telemetry.update();

        }

    }

    public double[] get_mecanum_powers(double drivex, double drivey) {
        double pfl, pfr, pbl, pbr;

        double turn = gamepad1.right_stick_x;

        double power = Math.hypot(drivex, drivey);
        double theta = Math.atan2(drivey, drivex);

        double sin = Math.sin(theta - Math.PI/4);
        double cos = Math.cos(theta - Math.PI/4);
        double max = Math.max(Math.abs(sin), Math.abs(cos));

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

        return new double[]{pfl, pfr, pbl, pbr};
    }
}
