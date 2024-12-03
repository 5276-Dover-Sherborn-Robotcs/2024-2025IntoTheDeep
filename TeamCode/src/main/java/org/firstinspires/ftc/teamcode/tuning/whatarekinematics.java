package org.firstinspires.ftc.teamcode.tuning;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;

import java.util.Arrays;

@Autonomous(name="WHAT THE FLIP ARE THE KINEMATICSSSS")
public class whatarekinematics extends LinearOpMode {

    DcMotorEx fl, fr, bl, br;
    DcMotorEx motors[];

    FtcDashboard dashboard = FtcDashboard.getInstance();

    @Override
    public void runOpMode() throws InterruptedException {

        telemetry = new MultipleTelemetry(telemetry, dashboard.getTelemetry());

        fl = hardwareMap.get(DcMotorEx.class, "m1");
        fr = hardwareMap.get(DcMotorEx.class, "m2");
        bl = hardwareMap.get(DcMotorEx.class, "m3");
        br = hardwareMap.get(DcMotorEx.class, "m4");

        motors = new DcMotorEx[]{fl, fr, bl, br};

        for (DcMotorEx motor : motors) {
            motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            motor.setTargetPositionTolerance(5);
            motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        }

        waitForStart();

        for (DcMotorEx motor : motors) {
            motor.setTargetPosition(5 * 28 * 20);
            motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        }

        while (Arrays.stream(motors).noneMatch(DcMotor::isBusy)) {

            TelemetryPacket packet = new TelemetryPacket();

            for (int i = 0; i < 4; i++) {
                packet.put(String.format("Motor %i current: ", i), motors[i].getCurrent(CurrentUnit.AMPS));
                packet.put(String.format("Motor %i position: ", i), (double)motors[i].getCurrentPosition());
            }

        }

        while (opModeIsActive()) {

            telemetry.addLine("WAITING");
            telemetry.update();

        }
    }
}
