package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.config.Config;

@Config
public class DriveConstants {

    public static double WHEEL_DIAMETER = 9.6;
    public static double WHEEL_CIRCUMFERENCE = WHEEL_DIAMETER * Math.PI;

    public static double MAX_VELOCITY_RPS = 87.85714285714286;
    public static double MAX_ACCELERATION_RPS = MAX_VELOCITY_RPS / 2;

    public static double GEAR_RATIO = 1/20.0;

    public static double MAX_VELOCITY = MAX_VELOCITY_RPS * GEAR_RATIO * WHEEL_CIRCUMFERENCE / Math.sqrt(2);
    public static double MAX_ACCELERATION = MAX_VELOCITY / 2;

    public static double Kv = 1 / MAX_VELOCITY;
    public static double Ka = Kv / 5.0;
    public static double Kp = 0;

}
