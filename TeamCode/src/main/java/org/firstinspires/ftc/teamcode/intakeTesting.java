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

    Servo l, r;
    
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
        l = hardwareMap.get(Servo.class, "l");
        l.setDirection(Servo.Direction.REVERSE);
        r = hardwareMap.get(Servo.class, "r");

        waitForStart();
        start_time = clock.seconds();

        boolean was_input = false;

        while (opModeIsActive()) {

            double power = gamepad1.left_stick_y / 2 + 0.5;
            l.setPosition(power);
            r.setPosition(power);

        }


    }

}
