package org.firstinspires.ftc.teamcode.controllers;

import static org.firstinspires.ftc.teamcode.DriveConstants.MAX_VELOCITY;

import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;

public class MecanumKinematics {

    // TODO: implement position PID control on top of motion profiling

    public static double FWD_GAIN = .5 / 5; // 50% power at 5 cm error .5 / 5
    public static double STR_GAIN = .25 / 5; // 25% power at 5 cm error .25 / 5
    public static double HEADING_GAIN = .25 / (Math.PI / 4); // 25% power at 45% error

    public double ly;
    public double lx;

    public MultipleTelemetry telemetry;

    // TODO: see if we need any of these literally ever
//    public ElapsedTime timer;
//    public double duration;
//    public double startTime;


    // TRACKWIDTH: distance between opposite wheels
    // WHEELBASE: distance between same side wheels
    public MecanumKinematics(double wheelbase, double trackwidth, MultipleTelemetry t) {
        ly = trackwidth / 2;
        lx = wheelbase / 2;
        telemetry = t;
    }

    public double[] compute(Pose2D error) {
        double drive = error.getX(DistanceUnit.CM) * FWD_GAIN;
        double strafe = error.getY(DistanceUnit.CM) * STR_GAIN;
        double turn = error.getHeading(AngleUnit.RADIANS) * HEADING_GAIN;
        return new double[]{
                drive - strafe - turn,
                drive + strafe + turn,
                drive + strafe - turn,
                drive - strafe + turn
        };
    }

//    public Pose2D forward(double[] deltas) {
//
//    }

    public double[] inverse(Pose2D velocity) {
        double[] robotvels = {velocity.getX(DistanceUnit.CM), velocity.getY(DistanceUnit.CM), velocity.getHeading(AngleUnit.RADIANS)};
        double[] motorvels = {0, 0, 0, 0};
        motorvels[0] = robotvels[0] - robotvels[1] - (lx + ly) * robotvels[2];
        motorvels[1] = robotvels[0] + robotvels[1] + (lx + ly) * robotvels[2];
        motorvels[2] = robotvels[0] + robotvels[1] - (lx + ly) * robotvels[2];
        motorvels[3] = robotvels[0] - robotvels[1] + (lx + ly) * robotvels[2];
        return motorvels;
    }

    public double[] limit_vels(double[] vels) {
        for (double vel : vels) {
            if (Math.abs(vel) > MAX_VELOCITY) {
                vel = Math.signum(vel) * MAX_VELOCITY;
            }
        }
        return vels;
    }

}
