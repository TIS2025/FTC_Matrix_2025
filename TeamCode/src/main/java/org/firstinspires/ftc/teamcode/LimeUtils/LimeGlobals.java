package org.firstinspires.ftc.teamcode.LimeUtils;

import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;

public class LimeGlobals {
    public static double corr = 0, yaw = 0, dist = 0;
    public static Pose3D botpose;
    public static double SPEED_GAIN  =  0.02  ;   //  Forward Speed Control "Gain". e.g. Ramp up to 50% power at a 25 inch error.   (0.50 / 25.0)
    public static double STRAFE_GAIN =  0.015 ;   //  Strafe Speed Control "Gain".  e.g. Ramp up to 37% power at a 25 degree Yaw error.   (0.375 / 25.0)
    public static double TURN_GAIN   =  0.025  ;   //  Turn Control "Gain".  e.g. Ramp up to 25% power at a 25 degree error. (0.25 / 25.0)

    public static  double MAX_AUTO_SPEED = 0.5;   //  Clip the approach speed to this max value (adjust for your robot)
    public static  double MAX_AUTO_STRAFE= 0.5;   //  Clip the strafing speed to this max value (adjust for your robot)
    public static  double MAX_AUTO_TURN  = 0.3;   //  Clip the turn speed to this max value (adjust for your robot)

    public static double h1 = 10.5, h2 = 29.5, a1 = 67, a2 = 0, angle_radian=0;
    public static double min = 0;
    public static double max = 0;
    public static int counts = 0;
    public static double y_pose;
    public static double x_pose;
    public static double z_pose;
    public static double roll;
    public static double pitch;
    public static double llYaw = 0, tx = 0 , ty = 0, tYaw = 0 ;
    public static boolean targetFound     = false;    // Set to true when an AprilTag target is detected
    public static double  driveS           = 0;        // Desired forward power/speed (-1 to +1)
    public static double  strafe          = 0;        // Desired strafe power/speed (-1 to +1)
    public static double  turn            = 0;        // Desired turning power/speed (-1 to +1)
    public static double DESIRED_DISTANCE = 50;
    public static double TargetYaw = 45;
    public static double rangeError;
    public static double offset = 13;
    public static double DESIRED_ANGLE = 130;
}
