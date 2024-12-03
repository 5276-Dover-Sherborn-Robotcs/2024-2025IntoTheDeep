package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.teamcode.DriveConstants.Ka;
import static org.firstinspires.ftc.teamcode.DriveConstants.Kv;
import static org.firstinspires.ftc.teamcode.DriveConstants.WHEEL_CIRCUMFERENCE;
import static org.firstinspires.ftc.teamcode.DriveConstants.WHEEL_DIAMETER;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;

public class Localizer {

    public enum dataType {
        TICKS,
        CENTIMETERS,
        RADIANS,
        MOTOR_RPM
    }

    public boolean DEBUGGING = false;

    FtcDashboard dashboard = FtcDashboard.getInstance();

    private double x0 = 0;
    private Pose2D starePose;
    private double theta = 0;
    private double[][] current_trajectory;
    private ElapsedTime trajectory_timer;

    private final double TICKS_PER_ROTATION = 28 * 5 * 4;
    private final double PI = Math.PI;
    private final double rt2 = Math.sqrt(2);

    public double current_time, previous_time, old_time, d_time = 0;

    private double gx, gy, heading = 0;

    private final DcMotorEx fl, fr, bl, br;

    private final DcMotorEx[] motors;

    private double[] old_encoders = {0, 0, 0, 0};
    private double[] prev_encoders = {0, 0, 0, 0};
    private double[] curr_encoders = {0, 0, 0, 0};
    private double[] delta_encoders = {0, 0, 0, 0};

    private Double[] wheel_velocities = {0.0, 0.0, 0.0, 0.0};

    private final Telemetry telemetry;

    public ElapsedTime timer = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);

    IMU imu;
    double prev_heading, d_heading;

    public Localizer(HardwareMap hardwareMap, Telemetry t) {

        telemetry = t;

        fl = hardwareMap.get(DcMotorEx.class, "m1");
        fr = hardwareMap.get(DcMotorEx.class, "m2");
        bl = hardwareMap.get(DcMotorEx.class, "m3");
        br = hardwareMap.get(DcMotorEx.class, "m4");

        fl.setDirection(DcMotorSimple.Direction.FORWARD);
        fr.setDirection(DcMotorSimple.Direction.REVERSE);
        bl.setDirection(DcMotorSimple.Direction.FORWARD);
        br.setDirection(DcMotorSimple.Direction.REVERSE);

        fl.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        fr.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        bl.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        br.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        fl.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        fr.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        bl.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        br.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        fl.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        fr.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        bl.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        br.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        imu = hardwareMap.get(IMU.class, "imu");

        RevHubOrientationOnRobot.LogoFacingDirection logoDirection = RevHubOrientationOnRobot.LogoFacingDirection.UP;
        RevHubOrientationOnRobot.UsbFacingDirection  usbDirection  = RevHubOrientationOnRobot.UsbFacingDirection.BACKWARD;

        RevHubOrientationOnRobot orientationOnRobot = new RevHubOrientationOnRobot(logoDirection, usbDirection);

        // Now initialize the IMU with this mounting orientation
        // Note: if you choose two conflicting directions, this initialization will cause a code exception.

        imu.initialize(new IMU.Parameters(orientationOnRobot));

        imu.resetYaw();
        heading = 0;
        prev_heading = 0;
        d_heading = 0;

        motors = new DcMotorEx[]{fl, fr, bl, br};

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
        prev_heading = heading;

        current_time = timer.time()/1000.0;
        d_time  = current_time - previous_time;
        old_time = previous_time;
        previous_time = current_time;

        for (int i = 0; i < 4; i++) {
            old_encoders[i] = prev_encoders[i];
            prev_encoders[i] = curr_encoders[i];
            curr_encoders[i] = motors[i].getCurrentPosition();
            delta_encoders[i] = curr_encoders[i] - prev_encoders[i];
            wheel_velocities[i] = (curr_encoders[i] - old_encoders[i]) / (current_time - old_time);
        }

        linearOdometry();

    }

    public void linearOdometry() {

        double dfwd = ((delta_encoders[0] + delta_encoders[1] + delta_encoders[2] + delta_encoders[3])  / 4.0) / TICKS_PER_ROTATION * WHEEL_CIRCUMFERENCE;
        double dstr = ((delta_encoders[1] + delta_encoders[2] - delta_encoders[3] - delta_encoders[0])  / 4.0) / TICKS_PER_ROTATION * WHEEL_CIRCUMFERENCE;

        double dx = dfwd * Math.cos(d_heading) - dstr * Math.sin(d_heading);
        double dy = dfwd * Math.sin(d_heading) + dstr * Math.cos(d_heading);

        // NEGATIVE BECAUSE IT WAS BACKWARDS??????
        // IT WAS BACKWARDS BECAUSE I FUCKED UP THE DELTAS
        gx = dx * Math.cos(prev_heading) - dy * Math.sin(prev_heading);
        gy = dx * Math.sin(prev_heading) + dy * Math.cos(prev_heading);
    }

    public void linearOdometry2() {

        double pair_fl = (delta_encoders[0] + delta_encoders[3]) / 2.0 / TICKS_PER_ROTATION / WHEEL_CIRCUMFERENCE;
        double pair_fr = (delta_encoders[1] + delta_encoders[2]) / 2.0 / TICKS_PER_ROTATION / WHEEL_CIRCUMFERENCE;

        double dfwd = pair_fl * Math.cos(PI/4) + pair_fr * Math.cos(-PI/4);
        double dstr = pair_fl * Math.sin(PI/4) + pair_fr * Math.sin(-PI/4);

        double dx = dfwd * Math.cos(d_heading) - dstr * Math.sin(d_heading);
        double dy = dfwd * Math.sin(d_heading) + dstr * Math.cos(d_heading);

        // NEGATIVE BECAUSE IT WAS BACKWARDS??????
        gx = dx * Math.cos(prev_heading) - dy * Math.sin(prev_heading);
        gy = dx * Math.sin(prev_heading) + dy * Math.cos(prev_heading);
    }

    public void arcOdometry() {

        double dfwd = ((delta_encoders[0] + delta_encoders[1] + delta_encoders[2] + delta_encoders[3])  / 4.0) / TICKS_PER_ROTATION * WHEEL_CIRCUMFERENCE;
        double dstr = ((delta_encoders[1] + delta_encoders[2] - delta_encoders[3] - delta_encoders[0])  / 4.0) / TICKS_PER_ROTATION * WHEEL_CIRCUMFERENCE;

        double r0 = dfwd/d_heading;
        double r1 = dstr/d_heading;

        double dx = r0 * Math.sin(d_heading) - r1 * (1 - Math.cos(d_heading));
        double dy = r1 * Math.sin(d_heading) + r0 * (1 - Math.cos(d_heading));

        gx = dx * Math.cos(prev_heading) - dy * Math.sin(prev_heading);
        gy = dx * Math.sin(prev_heading) + dy * Math.cos(prev_heading);
    }

    public void reset() {
        timer.reset();
    }

    public void generate_linear_trajectory(double x0, double x1, double max_accel, double max_veloc) {
        double dx = x1 - x0;
        if (max_veloc / max_accel < dx / max_veloc) {
            // normal trajectory
            double dt1 = max_veloc / max_accel;
            double dt2 = dx / max_veloc - max_veloc / max_accel;
            current_trajectory = new double[][]{
                    {max_accel, dt1},
                    {0, dt2},
                    {-max_accel, dt1}
            };
        } else {
            // degenerate trajectory
            double dt1 = Math.sqrt(dx / max_accel);
            current_trajectory = new double[][]{
                    {max_accel, dt1},
                    {-max_accel, dt1}
            };
        }
    }

    public void start_trajectory() {
        trajectory_timer = new ElapsedTime();
    }

    public void end_trajectory() {
        trajectory_timer = null;
        current_trajectory = null;
    }

    public double traj_pos_time() {
        double time = trajectory_timer.time();
        double x0 = this.x0;
        double v0 = 0;
        for (double[] phase : current_trajectory) {
            if (time < phase[1]) return x0 + v0 * time + phase[0] * time * time / 2;
            time -= phase[1];
            x0 += v0 * phase[1] + phase[0] * 0.5 * phase[1] * phase[1];
            v0 += phase[1] * phase[0];
        }
        return 0;
    }

    public double traj_vel_time() {
        double time = trajectory_timer.time();
        double v0 = 0;
        for (double[] phase : current_trajectory) {
            if (time < phase[1]) return v0 + phase[0] * time;
            time -= phase[1];
            v0 += phase[1] * phase[0];
        }
        return 0;
    }

    public double traj_acc_time() {
        double time = trajectory_timer.time();
        for (double[] phase : current_trajectory) {
            if (time < phase[1]) return phase[0];
            time -= phase[1];
        }
        return 0;
    }

    public boolean is_traj_done() {
        double time = trajectory_timer.time();
        for (double[] phase : current_trajectory) {
            time -= phase[1];
        }
        return (time >= 0);
    }

    public double[] traj_pos_vel_acc() {
        return new double[]{traj_pos_time(), traj_vel_time(), traj_acc_time()};
    }

    public double feedforward() {
        return Ka * traj_acc_time() + Kv * traj_vel_time();
    }

    public void telemetrize() {
        TelemetryPacket packet = new TelemetryPacket(false);
        packet.put("X POS", gx);
        packet.put("Y POS", gy);
        packet.put("HEADING", heading);
        packet.put("DELTA TIME", d_time);
        packet.addLine(String.format("WHEEL FL: %0.3f || WHEEL FR: %0.3f\nWHEEL BL: %0.3f || WHEEL BR: %0.3f", wheel_velocities[0], wheel_velocities[1], wheel_velocities[2], wheel_velocities[3]));
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

    public DcMotorEx[] getMotors() {
        return new DcMotorEx[]{fl, fr, bl, br};
    }

    public double[] getMotorPositions() {
        return curr_encoders;
    }

    public double[] getMotorDeltas() {
        return delta_encoders;
    }

    public double[] getMotorVelocities(dataType type) {
        double modifier;
        switch (type) {
            case TICKS:
                modifier = 1;
                break;
            case RADIANS:
                modifier = 2*PI/TICKS_PER_ROTATION;
                break;
            case CENTIMETERS:
                modifier = WHEEL_DIAMETER*PI/TICKS_PER_ROTATION;
                break;
            case MOTOR_RPM:
                modifier = 60/28.0;
                break;
            default:
                telemetry.addLine("INVALID DATATYPE FOR MOTOR VELOCITIES");
                return new double[]{};
        }
        return new double[]{motors[0].getVelocity()*modifier,motors[1].getVelocity()*modifier, motors[2].getVelocity()*modifier, motors[3].getVelocity()*modifier};
    }

    public double[] getMotorVelocities() {
        double modifier = 1/28.0;
        return new double[]{motors[0].getVelocity()*modifier,motors[1].getVelocity()*modifier, motors[2].getVelocity()*modifier, motors[3].getVelocity()*modifier};
    }

}
