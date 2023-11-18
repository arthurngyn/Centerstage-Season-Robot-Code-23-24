package org.firstinspires.ftc.teamcode.Hardware.Variables;

import com.acmerobotics.dashboard.config.Config;

@Config
public class AnglePID {
    public static double AKp = 1.8;  // Proportional gain
    public static double AKi = 0.0;   // Integral gain
    public static double AKd = 0;// Derivative gain

    public static double VKp = 0.03;
    public static double VKi = 0;

    public static double VKd = 0.000;


    public static double HORZ_ERROR = 0;
    public static double ANGLE_ERROR = 0;
    public static double HKp = 0.23;
    public static double HKi = 0;
    public static double HKd = 0.002;
    double integralSum = 0;
}
