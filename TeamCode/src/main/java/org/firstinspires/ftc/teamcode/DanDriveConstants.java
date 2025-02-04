package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.config.Config;

import org.firstinspires.ftc.teamcode.util.Pose2D;

@Config
public class DanDriveConstants {

    public static boolean DEBUGGING = false;

    public static double WHEEL_DIAMETER = 9.6/2.54;
    public static double WHEEL_CIRCUMFERENCE = WHEEL_DIAMETER * Math.PI;

    public static double MAX_VELOCITY_RPS = 5.1;
    public static double MAX_ACCELERATION_RPS = MAX_VELOCITY_RPS * 2;

    public static double GEAR_RATIO = 3/2.0;

    public static double MAX_VELOCITY = 60;
    public static double MAX_ACCELERATION = MAX_VELOCITY;

    public static double trackwidth = 15; // These values have been tuned (its weird ik (weight distribution))
    public static double wheelbase = 15;

    public static double MAX_ROTATIONAL_VELOCITY = 3.14;
    public static double MAX_ROTATIONAL_ACCELERATION = 3.14;

    public static double Kv = 0.010; // found experimentally then kicked down, pid shouldn't be pushing not pulling
    public static double Ka = 0.005;
    public static double Kp = 0;

    public static double INIT_ANGLE = 0.6;

    public static double TICKS_PER_ROTATION = 5281.1; // 5281.1 / 1.0 = x ticks / y rotations
    public static double TICKS_PER_INCH = 384.5 / (12.0/2.54); // 384.5 / 12.0 = x ticks / y cm

    public static double BASE_MAX_EXTENSION = 97.0/2.54;

    public static double Kg = 0.28; // x power at 0 degrees/ when sin(theta) is 1
    public static double Kgl = 4;
    public static double Kl = 0.02; // tuned

    public static double Bx = -0.5;

    public static double LATERAL_MULTIPLIER = 2;

    public static double SERVO_MULTIPLIER = 1800;

    public static double TEST_X = 24;
    public static double TEST_Y = 0;
    public static double TEST_H = 0;


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
