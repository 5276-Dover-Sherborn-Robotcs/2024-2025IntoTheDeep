package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.util.NanoClock;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

@Config
@TeleOp(name = "4 Bar Arm Testing")
public class intakeTesting extends LinearOpMode {

    Servo arm_pitch;
    
    double[] arm_pitch_positions = {90, 0, -90};
    int arm_pitch_index = 0;
    double arm_pitch_position = 90;
    public static double tuner = 1;
    public static double gravity_slow_down = 1;
    double arm_pitch_speed = 461.53/3;
    double p_val = 1/45.0;

    public static double g = 0.01;

    NanoClock clock = NanoClock.system();
    double prev_time = 0;
    double start_time = 0;

    @Override
    public void runOpMode() {

        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        arm_pitch = hardwareMap.get(Servo.class, "arm_pitch");
        arm_pitch.setDirection(Servo.Direction.REVERSE);

        waitForStart();
        start_time = clock.seconds();

        boolean was_input = false;

        while (opModeIsActive()) {

            double time = clock.seconds() - start_time;
            double dt = time - prev_time;

            // current_direction is used to determine whether the effect of gravity increases or decreases the speed of the servo
            double current_direction = -Math.signum((arm_pitch.getPosition() - 0.5) * 2);
            if (arm_pitch_position > 180) {
                current_direction = -current_direction;
            }
            if (current_direction > 0) current_direction *= gravity_slow_down;

            // speed is determined by what the base speed of the servo is, multiplied by a factor of 1 +- g
            double current_speed = tuner * arm_pitch_speed * (1 + current_direction * g * Math.cos(Math.toRadians(arm_pitch_position)));

            // Estimating its position based on estimated speed and current command
            arm_pitch_position += (arm_pitch.getPosition() - 0.5) * 2 * current_speed * dt;

            if (!was_input) {
                if (gamepad1.dpad_up && arm_pitch_index != arm_pitch_positions.length) arm_pitch_index++;
                else if (gamepad1.dpad_down && arm_pitch_index != 0) arm_pitch_index--;
            }

            was_input = gamepad1.dpad_up || gamepad1.dpad_down;

            double error = arm_pitch_positions[arm_pitch_index] - arm_pitch_position;
            double p = error * p_val + 0.5;

            arm_pitch.setPosition(Range.clip(p, 0, 1));

            telemetry.addData("Arm Pitch Position", arm_pitch_position);
            telemetry.addData("error", error);
            telemetry.addData("p", p);
            telemetry.addData("arm speed", current_speed);
            telemetry.addData("tuned speed", arm_pitch_speed);
            telemetry.update();

            prev_time = time;

        }


    }

}
