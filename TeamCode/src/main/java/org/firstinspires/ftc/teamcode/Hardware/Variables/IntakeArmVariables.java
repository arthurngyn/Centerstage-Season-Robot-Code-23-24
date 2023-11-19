package org.firstinspires.ftc.teamcode.Hardware.Variables;

import com.acmerobotics.dashboard.config.Config;

@Config
public class IntakeArmVariables {
    //ENCODERS

    public static int TRANSFER_PIXEL = 60;
    public static int INTAKE_PIXEL = -120;
    public static int HOVER_PIXEL = -60;
    public static int AVOID_PIXEL = -35;
    public static int START = 0;

    // PID
    public static double p = 0.0055, i =0, d=0.0003;
    public static double f =0.0;

    public static double pa = 0.0045, da = 0.0000;

    public static double GRAB_DISTANCE = 4.5;

    public static double maxVelocity = 0; // Max motor velocity
    public static double maxAcceleration = 0.0; // Max motor acceleration
    public static double maxJerk = 0.0; // Max motor jerk
}
