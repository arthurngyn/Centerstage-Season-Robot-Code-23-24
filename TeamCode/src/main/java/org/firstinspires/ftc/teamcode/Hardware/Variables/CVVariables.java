package org.firstinspires.ftc.teamcode.Hardware.Variables;

import com.acmerobotics.dashboard.config.Config;

import org.openftc.easyopencv.OpenCvCamera;

@Config
public class CVVariables {
    //Scalar Detector Values
    public static double lowerH = 0;
    public static double lowerS = 100;
    public static double lowerV = 100;
    public static double higherH = 100;
    public static double higherS = 255;
    public static double higherV = 255;

    public static final int CAMERA_WIDTH = 640; // width  of wanted camera resolution
    public static final int CAMERA_HEIGHT = 360; // height of wanted camera resolution

    // Calculate the distance using the formula
    public static final double objectWidthInRealWorldUnits = 3;  // Replace with the actual width of the object in real-world units
    public static final double focalLength = 720;  // Replace with the focal length of the camera in pixels
    public static final double widthFocalLength = 156;
}
