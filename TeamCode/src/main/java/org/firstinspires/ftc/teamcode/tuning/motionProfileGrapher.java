package org.firstinspires.ftc.teamcode.tuning;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.trajectories.DualLinearMotionProfile;
import org.firstinspires.ftc.teamcode.trajectories.MotionProfile;
import org.firstinspires.ftc.teamcode.util.Pose2D;

@Autonomous(name="Motion Profile Grapher")
public class motionProfileGrapher extends LinearOpMode {

    enum STATE {
        IDLE,
        RUNNING,
        DONE
    }

    private STATE state = STATE.IDLE;

    final double l = 15, b = 8;

    MotionProfile motionProfile;

    FtcDashboard dashboard = FtcDashboard.getInstance();

    ElapsedTime timer;

    @Override
    public void runOpMode() throws InterruptedException {

        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        motionProfile = new DualLinearMotionProfile(
                new Pose2D(0, 0, 0),
                new Pose2D(200, 200, Math.PI * 2));

        while (opModeInInit()) {
            telemetry.addLine("Localizer ready");
            telemetry.update();
        }
        timer = new ElapsedTime();

        while (opModeIsActive()) {
            switch (state) {
                case IDLE:
                    if (timer.time() >= 0.5) {
                        motionProfile.start();
                        state = STATE.RUNNING;
                        timer = null;
                    }

                    dashboard.sendTelemetryPacket(getTelemetryPacket(new Pose2D(0, 0, 0), new Pose2D(0, 0, 0), new Pose2D(0, 0, 0)));

                    break;
                case RUNNING:
                    Pose2D pos = motionProfile.trajectoryPosition();
                    Pose2D vel = motionProfile.trajectoryVelocity();
                    Pose2D accel = motionProfile.trajectoryAcceleration();

                    dashboard.sendTelemetryPacket(getTelemetryPacket(pos, vel, accel));

                    if (motionProfile.is_traj_done()) {
                        state = STATE.DONE;
                    }
                    break;
                case DONE:
                    motionProfile.end();
                    state = STATE.IDLE;
                    timer = new ElapsedTime();
                    break;
            }
            telemetry.update();

        }

    }

    private static @NonNull TelemetryPacket getTelemetryPacket(Pose2D pos, Pose2D vel, Pose2D accel) {
        TelemetryPacket packet = new TelemetryPacket(false);

        packet.put("Target X", pos.x);
        packet.put("Target Y", pos.y);
        packet.put("Target heading", pos.h);
        packet.put("Target X vel", vel.x);
        packet.put("Target Y vel", vel.y);
        packet.put("Target heading vel", vel.h);
        packet.put("Target X accel", accel.x);
        packet.put("Target Y accel", accel.y);
        packet.put("Target heading accel", accel.h);
        return packet;
    }
}
