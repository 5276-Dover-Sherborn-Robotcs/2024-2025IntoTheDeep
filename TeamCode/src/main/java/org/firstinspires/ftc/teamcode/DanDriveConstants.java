package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.config.Config;

@Config
public class DanDriveConstants {

    public static double WHEEL_DIAMETER = 7.5;
    public static double WHEEL_CIRCUMFERENCE = WHEEL_DIAMETER * Math.PI;

    public static double MAX_VELOCITY_RPS = 87.85714285714286;
    public static double MAX_ACCELERATION_RPS = MAX_VELOCITY_RPS / 2;

    public static double GEAR_RATIO = 1/20.0;

    public static double MAX_VELOCITY = MAX_VELOCITY_RPS * GEAR_RATIO * WHEEL_CIRCUMFERENCE; // / Math.sqrt(2);
    public static double MAX_ACCELERATION = MAX_VELOCITY / 4;

    public static double trackwidth = 0.1;
    public static double wheelbase = 0.1;

    public static double MAX_ROTATIONAL_VELOCITY = MAX_VELOCITY / (0.5 * trackwidth + 0.5 * wheelbase);
    public static double MAX_ROTATIONAL_ACCELERATION = MAX_ACCELERATION / (0.5 * trackwidth + 0.5 * wheelbase);

    public static double Kv = 1 / MAX_VELOCITY;
    public static double Ka = Kv / 5.0;
    public static double Kp = 0;

    public static double INIT_ANGLE = 0.6;

    public static double TICKS_PER_ROTATION = 3895.9; // 3895.9 / 1.0 = x ticks / y rotations
    public static double TICKS_PER_CM = 384.5 / 12.0; // 384.5 / 12.0 = x ticks / y cm

    public static double BASE_MAX_EXTENSION = 97.0;

    public static double Kg = 0.28; // x power at 0 degrees/ when sin(theta) is 1
    public static double Kgl = 3.5;
    public static double Kl = 0.25/BASE_MAX_EXTENSION;

    public static double Bx = 1;

}
