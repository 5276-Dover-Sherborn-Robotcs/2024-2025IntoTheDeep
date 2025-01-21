package org.firstinspires.ftc.teamcode.debugging;


import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.util.NanoClock;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.trajectories.DualLinearMotionProfile;
import org.firstinspires.ftc.teamcode.trajectories.LinearMotionProfile;
import org.firstinspires.ftc.teamcode.trajectories.MotionProfile;
import org.firstinspires.ftc.teamcode.util.Pose2D;

@Config
@TeleOp(name="Motion Profile Grapher (dashboard)")
public class motionProfileGrapher extends LinearOpMode {

    public static double DX = 0;
    public static double DY = 0;
    public static double DH = 0;

    public enum ProfileType {
        Linear,
        DualLinear;
    }
    public static ProfileType profileType = ProfileType.Linear;

    MotionProfile profile;

    NanoClock clock = NanoClock.system();
    double startTime;

    @Override
    public void runOpMode() throws InterruptedException {

        FtcDashboard.getInstance().setTelemetryTransmissionInterval(10);

        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        waitForStart();

        while (opModeIsActive()) {

            startTime = clock.seconds();

            if (gamepad1.cross || gamepad1.a) {

                telemetry.addLine("Creating Profile...");
                telemetry.update();

                switch (profileType) {
                    case Linear:
                        profile = new LinearMotionProfile(new Pose2D(0, 0, 0), new Pose2D(DX, DY, DH*Math.PI/180), telemetry);
                        break;
                    case DualLinear:
                        profile = new DualLinearMotionProfile(new Pose2D(0, 0, 0), new Pose2D(DX, DY, DH*Math.PI/180), telemetry);
                        break;
                }

                profile.start();

                telemetry.clear();

                telemetry.addLine("Running Profile...");
                telemetry.update();

                while (!profile.is_traj_done()) {

                    Pose2D[] forward = profile.get_time();

                    Pose2D position = forward[0];
                    Pose2D velocity = forward[1];
                    Pose2D acceleration = forward[2];

                    TelemetryPacket packet = new TelemetryPacket();

                    double[] x_points = {- 8.5, - 8.5, + 8.5, + 8.5};
                    double[] y_points = {- 8.5, + 8.5, + 8.5, - 8.5};

                    for (int i = 0; i < 4; i++) {

                        double x = x_points[i];
                        double y = y_points[i];

                        x_points[i] = x * Math.cos(position.h) - y * Math.sin(position.h) + position.x;
                        y_points[i] = y * Math.cos(position.h) + x * Math.sin(position.h) + position.y;

                    }

                    double[][] periods = profile.getTrajectory();
                    if (periods.length > 0) {
                        for (int i = 0; i < periods.length; i++) {

                            packet.put(String.format("Period %s X", i), periods[i][0]);
                            packet.put(String.format("Period %s H", i), periods[i][1]);
                            packet.put(String.format("Period %s dt", i), periods[i][2]);

                        }
                    }

                    packet.fieldOverlay()
                            .strokePolygon(x_points, y_points);

                    packet.put("X Position", position.x);
                    packet.put("Y Position", position.y);
                    packet.put("H Position", position.h);

                    packet.put("X Velocity", velocity.x);
                    packet.put("Y Velocity", velocity.y);
                    packet.put("H Velocity", velocity.h);

                    packet.put("X Acceleration", acceleration.x);
                    packet.put("Y Acceleration", acceleration.y);
                    packet.put("H Acceleration", acceleration.h);
                    
                    FtcDashboard.getInstance().sendTelemetryPacket(packet);

                }

                profile.end();
                profile = null;

            }



        }

    }
}
