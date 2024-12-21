package org.firstinspires.ftc.teamcode;


import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;

@Config
@TeleOp(name="Climb Testing")
public class climbTesting extends LinearOpMode {

    public static double p = 10.0;
    public static double i = 0.04998779296875;
    public static double d = 2.0;
    public static double f = 0.0;

    public static double INIT_ANGLE = 0.6; // degrees
    public static double ROTATION_POWER = 0.5;

    public double TICKS_PER_ROTATION = 3895.9; // 3895.9 / 1.0 = x ticks / y rotations
    public double TICKS_PER_CM = 384.5 / 12.0; // 384.5 / 12.0 = x ticks / y cm

    DcMotorEx arm_rot, left_extend, right_extend;

    FtcDashboard dashboard = FtcDashboard.getInstance();

    @Override
    public void runOpMode() {

        telemetry = new MultipleTelemetry(telemetry, dashboard.getTelemetry());

        arm_rot = hardwareMap.get(DcMotorEx.class, "m7");
        left_extend = hardwareMap.get(DcMotorEx.class, "m6");
        right_extend = hardwareMap.get(DcMotorEx.class, "m5");

        arm_rot.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        arm_rot.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        arm_rot.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        arm_rot.setDirection(DcMotorSimple.Direction.FORWARD);
        left_extend.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        left_extend.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        left_extend.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        left_extend.setDirection(DcMotorSimple.Direction.FORWARD);
        right_extend.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        right_extend.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        right_extend.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        right_extend.setDirection(DcMotorSimple.Direction.REVERSE);

        PIDFCoefficients pidf = new PIDFCoefficients(arm_rot.getPIDFCoefficients(DcMotor.RunMode.RUN_TO_POSITION));
        pidf.p = p;
        pidf.i = i;
        pidf.d = d;
        pidf.f = f;

        arm_rot.setPIDFCoefficients(DcMotor.RunMode.RUN_TO_POSITION, pidf);

        telemetry.addLine("Ready");
        telemetry.update();

        while (opModeInInit()) {

            TelemetryPacket packet = new TelemetryPacket(false);

            packet.put("Arm Rotation", String.format("Position: %d || Rotation: %.3f", arm_rot.getCurrentPosition(), arm_rot.getCurrentPosition()/TICKS_PER_ROTATION*360));
            double arm_pos = (left_extend.getCurrentPosition() + right_extend.getCurrentPosition())/2.0;
            packet.put("Arm Extension", String.format("Average: %.3f || Length: %.3f", arm_pos, arm_pos/TICKS_PER_CM));
            packet.put("P", pidf.p);
            packet.put("I", pidf.i);
            packet.put("D", pidf.d);
            packet.put("F", pidf.f);

            dashboard.sendTelemetryPacket(packet);

        }

        while (opModeIsActive()) {



            if (gamepad1.cross) {
                double target_rotation = (1.0 - INIT_ANGLE / 360) / 4.0;
                arm_rot.setTargetPosition(-(int) (target_rotation * TICKS_PER_ROTATION));
                arm_rot.setTargetPositionTolerance(2);
                arm_rot.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                arm_rot.setPower(ROTATION_POWER);
            }
            if (gamepad1.circle) {
                arm_rot.setTargetPosition(arm_rot.getCurrentPosition() + 100);
                arm_rot.setTargetPositionTolerance(2);
                arm_rot.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                arm_rot.setPower(ROTATION_POWER);
            }
            if (gamepad1.square) {
                arm_rot.setTargetPosition(arm_rot.getCurrentPosition() - 100);
                arm_rot.setTargetPositionTolerance(2);
                arm_rot.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                arm_rot.setPower(ROTATION_POWER);
            }
            if (gamepad1.triangle) {
                arm_rot.setTargetPosition(0);
                arm_rot.setTargetPositionTolerance(2);
                arm_rot.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                arm_rot.setPower(ROTATION_POWER);
            }

        }

    }

}