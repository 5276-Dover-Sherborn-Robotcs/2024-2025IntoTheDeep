package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

@TeleOp(name = "intaketesting")
public class intakeTesting extends LinearOpMode {

    Servo left, right;

    RevColorSensorV3 colorSensor;

    @Override
    public void runOpMode() {

        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        left = hardwareMap.get(Servo.class, "l");
        right = hardwareMap.get(Servo.class, "r");
        right.setDirection(Servo.Direction.REVERSE);

        colorSensor = hardwareMap.get(RevColorSensorV3.class, "color");
        colorSensor.setGain(1);

        telemetry.addLine("Ready");
        telemetry.update();
        waitForStart();

        while (opModeIsActive()) {

            double power = -gamepad1.left_stick_y/2 + 0.5;
            left.setPosition(power);
            right.setPosition(power);

            TelemetryPacket packet = new TelemetryPacket(false);

            packet.put("Red", colorSensor.red());
            packet.put("Blue", colorSensor.blue());
            packet.put("Green", colorSensor.green());
            packet.put("Alpha", colorSensor.alpha());
            packet.put("Distance", colorSensor.getDistance(DistanceUnit.CM));
            packet.put("Power", power);
            packet.put("Green/Blue Ratio", (double)colorSensor.green()/colorSensor.blue());
            packet.put("Green/Red Ratio", (double)colorSensor.green()/colorSensor.red());


            FtcDashboard.getInstance().sendTelemetryPacket(packet);

        }


    }

}
