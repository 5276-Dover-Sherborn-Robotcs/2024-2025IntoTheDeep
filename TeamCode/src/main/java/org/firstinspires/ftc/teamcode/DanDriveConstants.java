package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.config.Config;

import org.firstinspires.ftc.teamcode.util.Pose2D;

@Config
public class DanDriveConstants {

    public static double WHEEL_DIAMETER = 9.6;
    public static double WHEEL_CIRCUMFERENCE = WHEEL_DIAMETER * Math.PI;

    public static double MAX_VELOCITY_RPS = 5.2 * 0.9;
    public static double MAX_ACCELERATION_RPS = MAX_VELOCITY_RPS / 2;

    public static double GEAR_RATIO = 1.0;

    public static double MAX_VELOCITY = MAX_VELOCITY_RPS * GEAR_RATIO * WHEEL_CIRCUMFERENCE; // / Math.sqrt(2);
    public static double MAX_ACCELERATION = MAX_VELOCITY / 2;

    public static double trackwidth = (12 + 3.0/8)*2.54;
    public static double wheelbase = (11 + 7.0/8)*2.54;

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
    public static double Kl = 0.3/BASE_MAX_EXTENSION;

    public static double Bx = -0.5 * 2.54;


    // X AXIS IS THROUGH THE LONG SIDE OF THE SUBMERSIBLE
    // Y AXIS IS THROUGH THE SHORT SIDE, TOWARDS THE BLUE RUNGS
    /*


               POSITIVE Y
         ___________                        | blue bucket is up in this corner
        |    blue   |                       |
        |           |                       |
        |           | POSITIVE X            |
        |           |                       |
        |_____red___|                       | red observation zone is down here
              |
             \ /
            NEGATIVE Y

     */
    public static Pose2D BLUE_SIDE_SAMPLE_START = new Pose2D(32*2.54, 60*2.54, Math.toRadians(270));

}
