package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.util.NanoClock;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

@Config
@TeleOp(name = "4 Bar Arm Testing")
public class intakeTesting extends LinearOpMode {

    Servo intake_left, intake_right, intake_pitch, intake_roll, arm_pitch;

    NanoClock clock = NanoClock.system();
    double prev_time = 0;
    double start_time = 0;

    double intake_roll_pos = 0.0;
    public static double pitch_speed_control = 4; // tuned
    boolean flipped = false;
    double arm_pitch_pos = 1.0;
    double intake_pitch_pos = 1.0;

    public static double init_arm_pitch = 1.0;
    public static double init_intake_pitch = 1.0;

    @Override
    public void runOpMode() {

        // two pitch, one roll :D
        arm_pitch = hardwareMap.get(Servo.class, "arm_pitch");
        intake_pitch = hardwareMap.get(Servo.class, "intake_pitch");
        intake_roll = hardwareMap.get(Servo.class, "intake_roll");

        arm_pitch.setDirection(Servo.Direction.REVERSE);
        arm_pitch.scaleRange(0, 0.375);
        arm_pitch.setPosition(init_arm_pitch);
        intake_pitch.scaleRange(0, 0.45);
        intake_pitch.setPosition(init_intake_pitch);
        intake_roll.scaleRange(1/6.0 - .025, 5/6.0 + .025);
        intake_roll.setPosition(1.0);

        intake_left = hardwareMap.get(Servo.class, "left");
        intake_right = hardwareMap.get(Servo.class, "right");
        intake_left.setDirection(Servo.Direction.REVERSE);

        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        waitForStart();
        start_time = clock.seconds();

        while (opModeIsActive()) {

            double intake = (gamepad2.right_trigger - gamepad2.left_trigger) / 2.0 + 0.5;

            intake_left.setPosition(intake);
            intake_right.setPosition(intake);

            if (gamepad2.a && !flipped) {
                intake_roll_pos = 1 - intake_roll_pos;
                intake_roll.setPosition(intake_roll_pos);
            }
            flipped = gamepad2.a;

            double time = clock.seconds() - start_time;
            double dt = time - prev_time;

            arm_pitch_pos -= (gamepad2.left_stick_y / pitch_speed_control * dt);
            intake_pitch_pos -= (gamepad2.right_stick_y / pitch_speed_control * dt);

            if (Math.abs(intake_pitch_pos - arm_pitch_pos) > 0.25) {
                intake_pitch_pos = arm_pitch_pos + Math.copySign(0.25, intake_pitch_pos - arm_pitch_pos);
            }

            arm_pitch_pos = Math.min(1.0, Math.max(0, arm_pitch_pos));
            intake_pitch_pos = Math.min(1.0, Math.max(0, intake_pitch_pos));

            arm_pitch.setPosition(arm_pitch_pos);
            intake_pitch.setPosition(intake_pitch_pos);

            prev_time = time;

            telemetry.addData("Current Intake Roll Position", intake_roll.getPosition());
            telemetry.addData("Target Intake Roll Position", intake_roll_pos);

            telemetry.addData("Current Arm Pitch Position", (arm_pitch.getPosition() - 0.5) * 2 * 135);
            telemetry.addData("Target Arm Pitch Position", arm_pitch_pos);

            telemetry.addData("Current Intake Pitch Position", (intake_pitch.getPosition() - 0.5) * 2 * 135);
            telemetry.addData("Target Intake Pitch Position", intake_pitch_pos);

            telemetry.update();


        }


    }

}
