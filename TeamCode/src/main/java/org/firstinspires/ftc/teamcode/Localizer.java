package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
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

    private final double TICKS_PER_ROTATION = 28 * 5 * 4;
    private final double WHEEL_DIAMETER = 96;
    private final double PI = Math.PI;

    public double current_time, previous_time, d_time = 0;

    private double gx, gy, heading = 0;

    private final DcMotorEx fl, fr, bl, br;

    private double pe_fl, pe_fr, pe_bl, pe_br;
    private double e_fl, e_fr, e_bl, e_br;
    private double de_fl, de_fr, de_bl, de_br;

    private final Telemetry telemetry;

    public ElapsedTime timer = new ElapsedTime(ElapsedTime.Resolution.SECONDS);

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

        fl.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        fl.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        fr.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        fr.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        bl.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        bl.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        br.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        br.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        timer.reset();
        current_time = timer.time();

    }

    public void update() {
        // using constant velocity linear odometry
        // x += dx * cos(dtheta) - dy * sin(dtheta)
        // y += dy * cos(dtheta) + dx * sin(dtheta)
        // theta += dtheta

        current_time = timer.time();
        d_time  = current_time - previous_time;

        pe_fl = e_fl;
        pe_fr = e_fr;
        pe_bl = e_bl;
        pe_br = e_br;
        e_fl = fl.getCurrentPosition();
        e_fr = fr.getCurrentPosition();
        e_bl = bl.getCurrentPosition();
        e_br = br.getCurrentPosition();
        de_fl = e_fl - pe_fl;
        de_fr = e_fr - pe_fr;
        de_bl = e_bl - pe_bl;
        de_br = e_br - pe_br;


        double dx = (Math.sqrt(2) * (de_fl + de_fr + de_bl + de_br)) / 8.0 / TICKS_PER_ROTATION * WHEEL_DIAMETER * PI;
        double dy = (Math.sqrt(2) * (-de_fl + de_fr + de_bl - de_br)) / 8.0 / TICKS_PER_ROTATION * WHEEL_DIAMETER * PI;
        double dheading = ((de_fl + de_bl) - (de_fr + de_br)) / 4.0;

        gx += dx * Math.cos(dheading) - dy * Math.sin(dheading);
        gy += dx * Math.sin(dheading) + dy * Math.cos(dheading);
        heading = (heading + dheading) % (2*PI);
        previous_time = current_time;
    }

    public void telemetrize() {
        telemetry.addData("X POS", gx);
        telemetry.addData("Y POS", gy);
        telemetry.addData("HEADING", heading);
    }

    public double getTime() {
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
        return new double[]{e_fl, e_fr, e_bl, e_br};
    }

    public double[] getMotorDeltas() {
        return new double[]{de_fl, de_fr, de_bl, de_br};
    }

    public double[] getMotorVelocities(dataType type) {
        double modifier;
        switch (type) {
            case TICKS:
                modifier = 1;
                break;
            case RADIANS:
                modifier = 1/TICKS_PER_ROTATION;
                break;
            case CENTIMETERS:
                modifier = WHEEL_DIAMETER*PI/TICKS_PER_ROTATION;
                break;
            case MOTOR_RPM:
                modifier = 60/28.0;
                break;
            default:
                telemetry.addLine("INVALID OR NO DATATYPE FOR MOTOR VELOCITIES");
                return new double[]{};
        }
        return new double[]{de_fl/d_time*modifier, de_fr/d_time*modifier, de_bl/d_time*modifier, de_br/d_time*modifier};
    }

    public double[] getMotorAccelerations(dataType type) {
        double modifier;
        switch (type) {
            case TICKS:
                modifier = 1;
                break;
            case RADIANS:
                modifier = 1/TICKS_PER_ROTATION;
                break;
            case CENTIMETERS:
                modifier = WHEEL_DIAMETER*PI/TICKS_PER_ROTATION;
                break;
            case MOTOR_RPM:
                modifier = 60/28.0;
                break;
            default:
                telemetry.addLine("INVALID OR NO DATATYPE FOR MOTOR VELOCITIES");
                return new double[]{};
        }
        return new double[]{de_fl/d_time/d_time*modifier, de_fr/d_time/d_time*modifier, de_bl/d_time/d_time*modifier, de_br/d_time/d_time*modifier};
    }

}
