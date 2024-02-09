package org.firstinspires.ftc.teamcode.Hardware.Variables;

import com.acmerobotics.dashboard.config.Config;

import org.openftc.easyopencv.OpenCvCamera;

@Config
public class CVVariables {
    //Scalar Detector Values
    public static double lowerHB = 100;
    public static double lowerSB = 100;
    public static double lowerVB = 50;
    public static double higherHB = 140;
    public static double higherSB = 255;
    public static double higherVB = 255;

    //Scalar Detector Values
    public static double lowerHY = 0;
    public static double lowerSY= 100;
    public static double lowerVY = 100;
    public static double higherHY = 100;
    public static double higherSY = 255;
    public static double higherVY = 255;

    //Scalar Detector Values
    public static double lowerHR = 0;
    public static double lowerSR= 50;
    public static double lowerVR = 50;
    public static double higherHR = 10;
    public static double higherSR = 255;
    public static double higherVR = 255;

    public static final int CAMERA_WIDTH = 640; // width  of wanted camera resolution
    public static final int CAMERA_HEIGHT = 360; // height of wanted camera resolution

    // Calculate the distance using the formula
    public static final double objectWidthInRealWorldUnits = 3;  // Replace with the actual width of the object in real-world units
    public static final double focalLength = 720;  // Replace with the focal length of the camera in pixels
    public static final double widthFocalLength = 156;
}
