package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.teamcode.DanDriveConstants.*;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp(name="Single Motor Capped Acceleration Test")
public class SingularCappedAccel extends LinearOpMode {

    DcMotorEx motor;

    ElapsedTime timer = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);

    // Max Acceleration is in rps/s
    final double MAX_ACCEL = 100;

    enum state {
        IDLE,
        RUNNING,
        DONE
    }

    @Override
    public void runOpMode() throws InterruptedException {

        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        motor = hardwareMap.get(DcMotorEx.class, "m1");

        motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motor.setDirection(DcMotorSimple.Direction.FORWARD);
        motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        waitForStart();

        timer.reset();

        double prev_time = 0, output = 0;
        state STATE = state.IDLE;

        // FORMAT phases { phase: {dt, accel}, ... , {dt, accel} }
        double[][] trajectory = {
                {2, 50},
                {2, 0},
                {2, -50}
        };

        ElapsedTime trajectory_timer = null;

        while (opModeIsActive()) {

            double delta_time = timer.time() / 1000 - prev_time;
            prev_time += delta_time;

            // REWRITE AS A RATE LIMITER


            // todo: implement a motion profile (or multiple to see how it works)

            switch (STATE) {
                case IDLE:
                    if (gamepad1.a) {
                        STATE = state.RUNNING;
                        trajectory_timer = new ElapsedTime();
                    } else {

                        double target_output = -gamepad1.left_stick_y * 100;

                        //acceleration in rps
                        double accel = MAX_ACCEL * delta_time;
                        //output in rps
                        output += Math.max(Math.min(target_output - output, accel), -accel);

                        telemetry.addData("prev_output", output);
                        telemetry.addData("target_output", target_output);
                        telemetry.addData("difference", target_output - output);
                        telemetry.addData("accel", accel);
                        telemetry.addData("delta_time", delta_time);
                        telemetry.update();

                        motor.setPower(output / 100);
                    }
                    break;
                case RUNNING:

                    double dt = trajectory_timer.time();
                    double vt = get_traj_vel(trajectory, dt);
                    double at = get_traj_accel(trajectory, dt);

                    telemetry.addData("Target Velocty", vt);
                    telemetry.addData("Current Velocity", motor.getVelocity()/28);
                    telemetry.addData("Acceleration", at);
                    telemetry.update();

                    output = Kv * vt + Ka * at + Kp * (vt - motor.getVelocity()/28);

                    motor.setPower(output);

                    if (is_traj_done(trajectory, trajectory_timer.time())) STATE = state.DONE;
                    break;
                case DONE:
                    trajectory_timer = null;
                    STATE = state.IDLE;
                    break;
            }

            // We use rps instead of rpm in order to prevent making any error in the current rpm calculations 60x larger

//            double target_power = -gamepad1.left_stick_y;
//            double target_rps = target_power * 100;
//
//            position = motor.getCurrentPosition();
//
//            delta_position = position - prev_position;
//
//            double curr_rps = (delta_position / delta_time) / 28;
//            //double rps = a * curr_rps + (1 - a) * prev_rps;
//            double rps = motor.getVelocity() / 28;
//            prev_rps = rps;
//
//            double error_rps = target_rps - rps;
//            double sign = Math.signum(error_rps);
//
//            double accel_rps = MAX_ACCEL * delta_time * 60;
//
////            telemetry.addLine(String.format("target_power: %d\ntarget_rps: %d\nrps: %d\nerror_rpm: %d", target_power, target_rps, rps, error_rpm));
//            telemetry.addData("target_power", target_power);
//            telemetry.addData("target_rps", target_rps);
//            telemetry.addData("rps", rps);
//            telemetry.addData("error_rps", error_rps);
//            telemetry.addData("sign", sign);
//            telemetry.addData("accel_rps", accel_rps);
//            telemetry.addData("time", time);
//            telemetry.addData("delta_time", delta_time);
//
//            if (Math.abs(error_rps) < accel_rps) {
//                output_rps = target_rps;
//            } else {
//                output_rps = rps + sign * accel_rps;
//            }
//
//            output_power = Math.min(Math.max((output_rps / 100), -1), 1);
//
//            telemetry.addData("output_rps", output_rps);
//            telemetry.addData("output_power", output_power);
//
//            motor.setPower(output_power);
//
//            prev_position = position;
//            prev_time = time;
//
//            telemetry.update();

        }

    }

    private double get_traj_accel(double[][] trajectory, double dt) {
        for (double[] phase : trajectory) {
            if (dt < phase[0]) {
                return phase[1];
            }

            dt -= phase[0];
        }
        return 0;
    }

    private double get_traj_vel(double[][] trajectory, double dt) {
        double v0 = 0;
        for (double[] phase : trajectory) {
            if (dt < phase[0]) {
                return v0 + phase[1] * dt;
            }

            v0 += phase[1] * phase[0];
            dt -= phase[0];
        }
        return v0;
    }

    private double get_traj_pos(double[][] trajectory, double dt) {
        double v0 = 0;
        double x0 = 0;
        for (double[] phase : trajectory) {
            if (dt < phase[0]) {
                return x0 + v0 * dt + 0.5 * phase[1] * dt * dt;
            }

            x0 += v0 * phase[0] + 0.5 * phase[1] * phase[0] * phase[0];
            v0 += phase[1] * phase[0];
            dt -= phase[0];
        }
        return x0;
    }

    private boolean is_traj_done(double[][] trajectory, double dt) {
        double sum = 0;
        for (double[] phase : trajectory) sum += phase[0];
        return (dt > sum);
    }
}
