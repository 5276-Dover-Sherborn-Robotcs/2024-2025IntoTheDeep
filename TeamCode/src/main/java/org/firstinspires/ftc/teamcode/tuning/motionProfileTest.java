package org.firstinspires.ftc.teamcode.tuning;

import static org.firstinspires.ftc.teamcode.DanDriveConstants.Ka;
import static org.firstinspires.ftc.teamcode.DanDriveConstants.Kv;
import static org.firstinspires.ftc.teamcode.DanDriveConstants.LATERAL_MULTIPLIER;
import static org.firstinspires.ftc.teamcode.DanDriveConstants.TEST_H;
import static org.firstinspires.ftc.teamcode.DanDriveConstants.TEST_X;
import static org.firstinspires.ftc.teamcode.DanDriveConstants.TEST_Y;
import static org.firstinspires.ftc.teamcode.DanDriveConstants.trackwidth;
import static org.firstinspires.ftc.teamcode.DanDriveConstants.wheelbase;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Localizer;
import org.firstinspires.ftc.teamcode.trajectories.DualLinearMotionProfile;
import org.firstinspires.ftc.teamcode.util.Pose2D;

@Config
@TeleOp(name="Motion Profile Test")
public class motionProfileTest extends LinearOpMode {

    public static double PID_mult = 1;

    // PID Control constants to account for any error in position
    public static double FORWARD_GAIN = 0.1, Xd = 0.015, Xi = 0.01; // 30% power at 50 inches error
    public static double STRAFE_GAIN = 0.1, Yd = 0.015, Yi = 0.01;
    public static double ANG_GAIN = .5, Hd = 0.2, Hi = 0.015;

    public double prev_error_x = 0, x_sum = 0;
    public double prev_error_y = 0, y_sum = 0; // some pid control
    public double prev_error_h = 0, h_sum = 0;

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

    DualLinearMotionProfile motionProfile;

    Pose2D startPose = new Pose2D(0, 0, 0);

    @Override
    public void runOpMode() throws InterruptedException {

        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        localizer = new Localizer(hardwareMap, telemetry, startPose);

        motionProfile = new DualLinearMotionProfile(telemetry);

        fl = hardwareMap.get(DcMotorEx.class, "fl");
        fr = hardwareMap.get(DcMotorEx.class, "fr");
        bl = hardwareMap.get(DcMotorEx.class, "bl");
        br = hardwareMap.get(DcMotorEx.class, "br");

        fl.setDirection(DcMotorSimple.Direction.REVERSE);
        fr.setDirection(DcMotorSimple.Direction.FORWARD);
        bl.setDirection(DcMotorSimple.Direction.REVERSE);
        br.setDirection(DcMotorSimple.Direction.FORWARD);

        fl.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        fr.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        fl.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        fl.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        fr.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        fr.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        bl.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        bl.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        br.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        br.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        motors = new DcMotorEx[]{fl, fr, bl, br};

        while (opModeInInit()) {
            telemetry.addLine("Localizer ready");
            localizer.telemetrize();
            telemetry.addData("IMU ready", localizer.imu_ready);
            telemetry.update();
        }

        localizer.reset();

        while (opModeIsActive()) {
            localizer.telemetrize();
            localizer.update();

            boolean move = movePID(motionProfile);
            PID_mult = (gamepad1.y ? 1 : 0);

            switch (state) {
                case IDLE:
                    if (gamepad1.b) {
                        motionProfile = new DualLinearMotionProfile(
                                startPose,
                                new Pose2D(TEST_X, TEST_Y, (TEST_H * Math.PI/180)).plus(startPose),
                                telemetry);
                        motionProfile.start();
                        state = STATE.RUNNING;
                    }
                    if (gamepad1.a) {
                        motionProfile = new DualLinearMotionProfile(
                                startPose,
                                new Pose2D(TEST_X, TEST_Y, TEST_H * Math.PI/180),
                                telemetry);
                        motionProfile.start();
                        state = STATE.RUNNING;
                    }

                    break;
                case RUNNING:

                    if (move) {
                        state = STATE.RESET;
                    }

                    break;
                case DONE:
                    break;
                case RESET:

                    fl.setPower(0);
                    fr.setPower(0);
                    bl.setPower(0);
                    br.setPower(0);

                    ElapsedTime timer = new ElapsedTime();
                    while (timer.time() < 3) {
                        telemetry.update();
                        telemetry.addLine("Motion Profile Complete");
                    }
                    state = STATE.IDLE;
                    startPose = motionProfile.getEndPose();
                    motionProfile.end();
                    motionProfile = new DualLinearMotionProfile(startPose, telemetry);
                    break;
            }

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

    public boolean movePID(DualLinearMotionProfile profile) {
        // profile.get_time() returns a pose for position, a pose for velocity, and a pose for acceleration in an array of poses, in that order.
        Pose2D[] at_time = profile.get_state_at_time();

        Pose2D poseTarget = at_time[0];
        Pose2D poseEstimate = localizer.getPoseEstimate();

        // PID control to account for error in where we are over time.

        double error_x = poseTarget.x - poseEstimate.x;
        double error_y = poseTarget.y - poseEstimate.y;
        double error_h = poseTarget.h - poseEstimate.h;

        if (error_h < -2 * Math.PI) error_h += 2 * Math.PI;

        if ((2 * Math.PI - Math.abs(error_h)) < Math.abs(error_h)) {
            error_h = -Math.copySign(2 * Math.PI - Math.abs(error_h), error_h);
        }

        // P gains
        double forward = error_x * FORWARD_GAIN;
        double strafe = error_y * STRAFE_GAIN;
        double turn = error_h * ANG_GAIN;

        // I gains

        if (Math.abs(error_h) < 0.1) h_sum = 0;
        if (Math.abs(error_x) < 0.1) x_sum = 0;
        if (Math.abs(error_y) < 0.1) y_sum = 0;

        x_sum += error_x * PID_mult;
        if (x_sum * error_x < 0) x_sum = 0;
        forward += x_sum * Xi;
        y_sum += error_y * PID_mult;
        if (y_sum * error_y < 0) y_sum = 0;
        strafe += y_sum * Yi;
        h_sum += error_h * PID_mult;
        if (h_sum * error_h < 0) h_sum = 0;
        turn += h_sum * Hi;

        // D gains
        forward += (error_x - prev_error_x) / localizer.d_time * Xd;
        strafe += (error_y - prev_error_y) / localizer.d_time * Yd;
        turn += (error_h - prev_error_h) / localizer.d_time * Hd;

        // Rotate it
        forward = forward * Math.cos(-poseEstimate.h) - strafe * Math.sin(-poseEstimate.h);
        strafe = forward * Math.sin(-poseEstimate.h) + strafe * Math.cos(-poseEstimate.h);

        // Feedforward control
        Pose2D vel = at_time[1];
        Pose2D accel = at_time[2];

        double x = vel.x * Kv + accel.x * Ka + forward * PID_mult;
        double y = vel.y * Kv + accel.y * Ka + strafe * PID_mult;
        y *= LATERAL_MULTIPLIER;
        double w = (vel.h * Kv + accel.h * Ka) * (trackwidth/2 + wheelbase/2) + turn * PID_mult;

        double pfl = x - y - w;
        double pfr = x + y + w;
        double pbl = x + y - w;
        double pbr = x - y + w;

        fl.setPower(pfl);
        fr.setPower(pfr);
        bl.setPower(pbl);
        br.setPower(pbr);

        double TICKS_PER_INCH = 2000 / (4.8 / 2.54 * Math.PI);

        double[] current_velocity = localizer.getVelocity();

        TelemetryPacket packet = new TelemetryPacket();

        packet.put("X Velocity", vel.x);
        packet.put("Measured X Velocity", current_velocity[0] / TICKS_PER_INCH);
        packet.put("Y Velocity", vel.y);
        packet.put("Measured Y Velocity", current_velocity[1] / TICKS_PER_INCH);
        packet.put("W Velocity", vel.h);
        packet.put("Measured W Velocity", current_velocity[2]);
        packet.put("X Accel", accel.x);
        packet.put("Y Accel", accel.y);
        packet.put("W Accel", accel.h);
        packet.put("Forward", forward);
        packet.put("Strafe", strafe);
        packet.put("Heading", turn);
        Pose2D temp = profile.getStartPose();
        packet.put("Profile Start Pose X", temp.x);
        packet.put("Profile Start Pose Y", temp.y);
        packet.put("Profile Start Pose H", temp.h);
        temp = profile.getEndPose();
        packet.put("Profile End Pose X", temp.x);
        packet.put("Profile End Pose Y", temp.y);
        packet.put("Profile End Pose H", temp.h);
        packet.put("Target Pose X", at_time[0].x);
        packet.put("Target Pose Y", at_time[0].y);
        packet.put("Target Pose H", at_time[0].h);

        FtcDashboard.getInstance().sendTelemetryPacket(packet);

        prev_error_x = error_x;
        prev_error_y = error_y;
        prev_error_h = error_h;

        double total_power = Math.abs(x) + Math.abs(y) + Math.abs(w);

        return profile.is_traj_done() && total_power < 0.1;
    }
}
