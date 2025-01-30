package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.teamcode.DanDriveConstants.Bx;
import static org.firstinspires.ftc.teamcode.DanDriveConstants.trackwidth;
import static org.firstinspires.ftc.teamcode.DanDriveConstants.wheelbase;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.canvas.Canvas;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.util.Encoder;
import org.firstinspires.ftc.teamcode.util.Pose2D;

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
    private final double WHEEL_CIRCUMFERENCE = 4.8 / 2.54 * PI;

    private final double TICKS_PER_INCH = TICKS_PER_ROTATION / WHEEL_CIRCUMFERENCE;

    public static double X_MULTIPLIER = 1;
    public static double Y_MULTIPLIER = 1;

    public double current_time, previous_time, d_time = 0;

    private double gx = 0, gy = 0, heading = 0;

    Encoder X, Y;
    double old_X = 0, old_Y = 0;
    double dX = 0, dY = 0;

    public ElapsedTime timer = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);

    Telemetry telemetry;

    public IMU imu;
    public boolean imu_ready = false;
    double prev_heading, d_heading;

    public Localizer(HardwareMap hardwareMap, Telemetry t) {

        telemetry = t;

        X = new Encoder(hardwareMap.get(DcMotorEx.class, "fl"));
        Y = new Encoder(hardwareMap.get(DcMotorEx.class, "fr"));

        X.setDirection(Encoder.Direction.REVERSE);
        Y.setDirection(Encoder.Direction.REVERSE);

        imu = hardwareMap.get(IMU.class, "imu");

        RevHubOrientationOnRobot.LogoFacingDirection logoDirection = RevHubOrientationOnRobot.LogoFacingDirection.RIGHT;
        RevHubOrientationOnRobot.UsbFacingDirection  usbDirection  = RevHubOrientationOnRobot.UsbFacingDirection.UP;

        RevHubOrientationOnRobot orientationOnRobot = new RevHubOrientationOnRobot(logoDirection, usbDirection);

        // Now initialize the IMU with this mounting orientation
        // Note: if you choose two conflicting directions, this initialization will cause a code exception.

        imu_ready = imu.initialize(new IMU.Parameters(orientationOnRobot));

        imu.resetYaw();
        heading = 0;
        gx = 0;
        gy = 0;
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
        d_heading = heading - prev_heading;

        if (Math.abs(d_heading) > Math.PI) {
            d_heading = -Math.copySign(2*Math.PI - Math.abs(d_heading), d_heading);
        }

        current_time = timer.time()/1000.0;
        d_time  = current_time - previous_time;

        double x = X.getCurrentPosition();
        dX = x - old_X;
        old_X = x;
        double y = Y.getCurrentPosition();
        dY = y - old_Y;
        old_Y = y;

        if (d_heading > 1e-2) arcOdometry();
        else linearOdometry();

        previous_time = current_time;
        prev_heading = heading;

    }

    public void whatisturninganyways() {

        double dfwd = dX / TICKS_PER_INCH;
        double dstr = dY / TICKS_PER_INCH;

        gx += (dfwd * Math.cos(prev_heading) - dstr * Math.sin(prev_heading));
        gy += (dfwd * Math.sin(prev_heading) + dstr * Math.cos(prev_heading));

    }

    public void linearOdometry() {

        double dfwd = dX / TICKS_PER_INCH;
        double dstr = dY / TICKS_PER_INCH - Bx * d_heading;

        double dx = dfwd * Math.cos(d_heading) - dstr * Math.sin(d_heading);
        double dy = dfwd * Math.sin(d_heading) + dstr * Math.cos(d_heading);

        // todo: never fuck up the deltas
        // I FRICKIN SCREWED UP THE DELTAS AGAIN (i left in the / 4.0 from the previous averaging)

        gx += (dx * Math.cos(prev_heading) - dy * Math.sin(prev_heading));
        gy += (dx * Math.sin(prev_heading) + dy * Math.cos(prev_heading));
    }

    public void arcOdometry() {

        double dfwd = dX / TICKS_PER_INCH;
        double dstr = dY / TICKS_PER_INCH - Bx * d_heading;

        double r0 = dfwd/d_heading;
        double r1 = dstr/d_heading;

        double dx = r0 * Math.sin(d_heading) - r1 * (1 - Math.cos(d_heading));
        double dy = r1 * Math.sin(d_heading) + r0 * (1 - Math.cos(d_heading));

        gx += (dx * Math.cos(prev_heading) - dy * Math.sin(prev_heading));
        gy += (dx * Math.sin(prev_heading) + dy * Math.cos(prev_heading));
    }

    public void poseExponential() {

        double dfwd = dX / TICKS_PER_INCH;
        double dstr = dY / TICKS_PER_INCH - Bx * d_heading;

        double dx;
        double dy;

        if (d_heading > 1e-2) {

            dx = (Math.sin(d_heading)/d_heading) * dfwd + ((Math.cos(d_heading) - 1)/d_heading) * dstr;
            dy = (1 - (Math.cos(d_heading))/d_heading) * dfwd + (Math.sin(d_heading)/d_heading) * dstr;

        } else {

            dx = (1 - (d_heading * d_heading / 6)) * dfwd + (-d_heading/2) * dstr;
            dy = (1 - (d_heading * d_heading / 6)) * dstr + (d_heading/2) * dfwd;

        }

        gx += (dx * Math.cos(prev_heading) - dy * Math.sin(prev_heading));
        gy += (dx * Math.sin(prev_heading) + dy * Math.cos(prev_heading));

    }

    public void setPoseEstimate(Pose2D pose) {
        gx = pose.x;
        gy = pose.y;
        heading = pose.h;
    }

    public void reset() {

        gx = 0;
        gy = 0;

        timer.reset();
    }

    public void telemetrize() {
        TelemetryPacket packet = new TelemetryPacket();

        if (DEBUGGING) {

            Canvas canvas = packet.field();

            double[] x_points = {0, wheelbase/2, wheelbase/2, -wheelbase/2, -wheelbase/2, wheelbase/2, wheelbase/2};
            double[] y_points = {0, 0, trackwidth/2, trackwidth/2, -trackwidth/2, -trackwidth/2, 0};

            for (int i = 0; i < 7; i++) {

                double x = x_points[i] * Math.cos(heading) - y_points[i] * Math.sin(heading);
                double y = x_points[i] * Math.sin(heading) + y_points[i] * Math.cos(heading);

                x_points[i] = x;
                y_points[i] = y;

            }

            canvas.strokePolygon(x_points, y_points);

        }

        packet.put("X POS", gx);
        packet.put("Y POS", gy);
        packet.put("HEADING", heading);
        packet.put("HERTZ", 1/d_time);
        packet.put("d_heading", d_heading);
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
        return new Pose2D(gx, gy, heading);
    }

    public double[] getDeltas() {
        return new double[]{dX, dY};
    }

    public double[] getVelocity() {
        return new double[]{
                X.getCorrectedVelocity(),
                Y.getCorrectedVelocity(),
                imu.getRobotAngularVelocity(AngleUnit.RADIANS).zRotationRate
        };
    }

}
