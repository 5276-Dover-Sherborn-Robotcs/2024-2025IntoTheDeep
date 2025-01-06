package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.teamcode.DanDriveConstants.Bx;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode.util.Encoder;

public class Localizer {

    public enum dataType {
        TICKS,
        CENTIMETERS,
        RADIANS,
        MOTOR_RPM
    }

    public boolean DEBUGGING = false;

    FtcDashboard dashboard = FtcDashboard.getInstance();

    private final double PI = Math.PI;
    private final double rt2 = Math.sqrt(2);

    private final double TICKS_PER_ROTATION = 2000;
    private final double WHEEL_CIRCUMFERENCE = 4.8 * PI;

    private final double TICKS_PER_CM = TICKS_PER_ROTATION / WHEEL_CIRCUMFERENCE;

    public static double X_MULTIPLIER = 1;
    public static double Y_MULTIPLIER = 1;

    public double current_time, previous_time, old_time, d_time = 0;

    private double gx, gy, heading = 0;

    private final Encoder F, L;
    private final Encoder[] encoders;

    private double[] prev_encoders = {0, 0};
    private double[] curr_encoders = {0, 0};
    private double[] deltas = {0, 0};

    public ElapsedTime timer = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);

    Telemetry telemetry;

    public IMU imu;
    public boolean imu_ready = false;
    double prev_heading, d_heading;

    public Localizer(HardwareMap hardwareMap, Telemetry t) {

        telemetry = t;

        F = new Encoder(hardwareMap.get(DcMotorEx.class, "fl"));
        L = new Encoder(hardwareMap.get(DcMotorEx.class, "fr"));

        F.setDirection(Encoder.Direction.REVERSE);
        L.setDirection(Encoder.Direction.REVERSE);

        encoders = new Encoder[]{F, L};

        imu = hardwareMap.get(IMU.class, "imu");

        RevHubOrientationOnRobot.LogoFacingDirection logoDirection = RevHubOrientationOnRobot.LogoFacingDirection.RIGHT;
        RevHubOrientationOnRobot.UsbFacingDirection  usbDirection  = RevHubOrientationOnRobot.UsbFacingDirection.UP;

        RevHubOrientationOnRobot orientationOnRobot = new RevHubOrientationOnRobot(logoDirection, usbDirection);

        // Now initialize the IMU with this mounting orientation
        // Note: if you choose two conflicting directions, this initialization will cause a code exception.

        imu_ready = imu.initialize(new IMU.Parameters(orientationOnRobot));

        imu.resetYaw();
        heading = 0;
        prev_heading = 0;
        d_heading = 0;

        timer.reset();
        current_time = 0;

    }

    public void update() {
        // using constant velocity linear odometry
        // x += dx * cos(dtheta) - dy * sin(dtheta)
        // y += dy * cos(dtheta) + dx * sin(dtheta)
        // theta += dtheta

        heading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);
        heading += (heading < 0) ? 2*PI : 0;
        d_heading  = heading - prev_heading;

        current_time = timer.time()/1000.0;
        d_time  = current_time - previous_time;
        old_time = previous_time;

        for (int i = 0; i < encoders.length; i++) {
            curr_encoders[i] = encoders[i].getCurrentPosition();
            deltas[i] = curr_encoders[i] - prev_encoders[i];
        }

        if (Math.abs(d_heading) < 1e-3) {
            linearOdometry();
        } else {
            linearOdometry();
        }

        previous_time = current_time;
        prev_heading = heading;

        System.arraycopy(curr_encoders, 0, prev_encoders, 0, encoders.length);

    }

    public void linearOdometry() {

        double dfwd = deltas[0] / TICKS_PER_CM;
        double dstr = deltas[1] / TICKS_PER_CM - Bx * d_heading;

        double dx = dfwd * Math.cos(d_heading) - dstr * Math.sin(d_heading);
        double dy = dfwd * Math.sin(d_heading) + dstr * Math.cos(d_heading);

        // todo: never fuck up the deltas
        // I FRICKIN SCREWED UP THE DELTAS AGAIN (i left in the / 4.0 from the previous averaging)

        gx += (dx * Math.cos(prev_heading) - dy * Math.sin(prev_heading));
        gy += (dx * Math.sin(prev_heading) + dy * Math.cos(prev_heading));
    }

    public void arcOdometry() {

        double dfwd = deltas[0] / TICKS_PER_CM;
        double dstr = deltas[1] / TICKS_PER_CM - Bx * d_heading;

        double r0 = dfwd/d_heading;
        double r1 = dstr/d_heading;

        double dx = r0 * Math.sin(d_heading) - r1 * (1 - Math.cos(d_heading));
        double dy = r1 * Math.sin(d_heading) + r0 * (1 - Math.cos(d_heading));

        gx += (dx * Math.cos(prev_heading) - dy * Math.sin(prev_heading));
        gy += (dx * Math.sin(prev_heading) + dy * Math.cos(prev_heading));
    }

    public void poseExponential() {

        double dfwd = deltas[0] / TICKS_PER_CM;
        double dstr = deltas[1] / TICKS_PER_CM - Bx * d_heading;

        double dx;
        double dy;

        if (d_heading > 1e-2) {

            dx = (Math.sin(d_heading)/d_heading) * dfwd + ((Math.cos(d_heading) - 1)/d_heading) * dstr;
            dy = (Math.sin(d_heading)/d_heading) * dstr + (1 - (Math.cos(d_heading))/d_heading) * dfwd;

        } else {

            dx = (1 - (d_heading * d_heading / 6)) * dfwd + (-d_heading/2) * dstr;
            dy = (1 - (d_heading * d_heading / 6)) * dstr + (d_heading/2) * dfwd;

        }

        gx += (dx * Math.cos(prev_heading) - dy * Math.sin(prev_heading));
        gy += (dx * Math.sin(prev_heading) + dy * Math.cos(prev_heading));

    }

    public void reset() {

        gx = 0;
        gy = 0;
        deltas[0] = 0;
        deltas[1] = 0;

        timer.reset();
    }

    public void telemetrize() {
        TelemetryPacket packet = new TelemetryPacket(false);
        packet.put("X POS", gx);
        packet.put("Y POS", gy);
        packet.put("HEADING", heading);
        packet.put("DELTA TIME", d_time);
        dashboard.sendTelemetryPacket(packet);
    }

    public double time() {
        return timer.time();
    }

    public double getX() {
        return gx;
    }

    public double getY() {
        return gy;
    }

    public double getHeading() {
        return heading;
    }

    public Pose2D getPoseEstimate() {
        return new Pose2D(DistanceUnit.CM, gx, gy, AngleUnit.RADIANS, heading);
    }

    public double[] getDeltas() {
        return deltas;
    }

}
