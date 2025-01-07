package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.teamcode.DanDriveConstants.Ka;
import static org.firstinspires.ftc.teamcode.DanDriveConstants.Kv;
import static org.firstinspires.ftc.teamcode.DanDriveConstants.trackwidth;
import static org.firstinspires.ftc.teamcode.DanDriveConstants.wheelbase;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.trajectories.LinearMotionProfile;
import org.firstinspires.ftc.teamcode.trajectories.MotionProfile;
import org.firstinspires.ftc.teamcode.util.Pose2D;

@TeleOp(name="Motion Profile Test")
public class motionProfileTest extends LinearOpMode {

    enum STATE {
        IDLE,
        RUNNING,
        DONE,
        RESET
    }

    private STATE state = STATE.IDLE;

    Localizer localizer;

    DcMotorEx[] motors;

    DcMotorEx fl, fr, bl, br;

    double[] outputs = {0, 0, 0, 0};

    static double lx = trackwidth/2.0, ly = wheelbase/2.0;

    MotionProfile motionProfile;

    @Override
    public void runOpMode() throws InterruptedException {

        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        localizer = new Localizer(hardwareMap, telemetry);

        motionProfile = new LinearMotionProfile(
                new Pose2D(0, 0, 0),
                new Pose2D(24*2.54, 24*2.54, 0),
                telemetry);

        IMU imu = localizer.imu;

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

        while (opModeInInit()) {
            telemetry.addLine("Localizer ready");
            localizer.telemetrize();
            telemetry.addData("IMU ready", localizer.imu_ready);
            telemetry.update();
        }

        localizer.reset();
        boolean wasy = false;
        double timey = 0.0;

        while (opModeIsActive()) {
            localizer.telemetrize();
            motionProfile.telemetrize();
            localizer.update();

            switch (state) {
                case IDLE:

                    if (gamepad1.y) {
                        if (!wasy) {
                            timey = localizer.time();
                        }
                        if (wasy && timey != 0.0 && (localizer.time() - timey >= 2)) {
                            localizer.DEBUGGING = !localizer.DEBUGGING;
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
                        double accel = 2 * delta_time * ((max_power != 0) ? Math.abs(target_powers[i] / max_power) : 1);

                        double difference = target_powers[i] - outputs[i];

                        outputs[i] += Math.max(Math.min(difference, accel), -accel);
                        motors[i].setPower(outputs[i]);
                    }

                    if (gamepad1.a) {
                        // todo: start a forward motion profiling

//                        motionProfile = new DualLinearMotionProfile(
//                                new Pose2D(0, 0, 0),
//                                new Pose2D(60, 20, 0),
//                                telemetry);
                        motionProfile.start();

                        state = STATE.RUNNING;
                    } else if (gamepad1.y) {
                        state = STATE.RESET;
                    }

                    break;
                case RUNNING:
                    Pose2D vel = motionProfile.traj_vel_time();
                    Pose2D accel = motionProfile.traj_acc_time();

                    double x = Kv * vel.x + Ka * accel.x;
                    double y = Kv * vel.y + Ka * accel.y;
                    double w = (Kv * vel.h + Ka * accel.h) * (lx + ly);

                    telemetry.addData("x and y", "%f, %f", x, y);

                    outputs[0] = x - y - w;
                    outputs[1] = x + y + w;
                    outputs[2] = x + y - w;
                    outputs[3] = x - y + w;
//                    outputs[0] = x;
//                    outputs[1] = y;
//                    outputs[2] = y;
//                    outputs[3] = x;

                    for (int i = 0; i < 4; i++) {
                        motors[i].setPower(outputs[i]);
                    }

                    if (motionProfile.is_traj_done()) {
                        state = STATE.DONE;
                    }
                    break;
                case DONE:
                    for (int i = 0; i < 4; i++) motors[i].setPower(0);
                    motionProfile.end();
                    state = STATE.IDLE;
                    break;
                case RESET:
                    for (int i = 0; i < 4; i++) motors[i].setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                    ElapsedTime timer = new ElapsedTime();
                    while (timer.time() < 3) {
                        telemetry.update();
                        telemetry.addLine("RESETTING ENCODERS");
                    }
                    for (int i = 0; i < 4; i++) motors[i].setMode(DcMotor.RunMode.RUN_USING_ENCODER);
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
