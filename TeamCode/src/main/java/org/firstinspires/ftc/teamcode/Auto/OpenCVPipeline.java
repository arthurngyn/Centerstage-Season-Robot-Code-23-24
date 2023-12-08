package org.firstinspires.ftc.teamcode.Auto;

import static org.firstinspires.ftc.teamcode.Hardware.Variables.CVVariables.higherHY;
import static org.firstinspires.ftc.teamcode.Hardware.Variables.CVVariables.higherSY;
import static org.firstinspires.ftc.teamcode.Hardware.Variables.CVVariables.higherVY;
import static org.firstinspires.ftc.teamcode.Hardware.Variables.CVVariables.lowerHY;
import static org.firstinspires.ftc.teamcode.Hardware.Variables.CVVariables.lowerHY;
import static org.firstinspires.ftc.teamcode.Hardware.Variables.CVVariables.lowerSY;
import static org.firstinspires.ftc.teamcode.Hardware.Variables.CVVariables.lowerVY;

import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.core.Size;
import org.opencv.imgproc.Imgproc;
import org.opencv.imgproc.Moments;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvPipeline;

import java.util.ArrayList;
import java.util.List;

public class OpenCVPipeline extends OpenCvPipeline {
    private Mat hsvFrame = new Mat();
    private Mat yellowMask = new Mat();
    private List<MatOfPoint> contours = new ArrayList<>();
    private Scalar color = new Scalar(255, 0, 0);
    private Scalar lowerYellow = new Scalar(lowerHY, lowerSY, lowerVY);
    private Scalar upperYellow = new Scalar(higherHY, higherSY, higherVY);
    private double cX = 0;
    private double cY = 0;
    double width = 0;
    public static double x = 0;
    public static double y = 0;

    @Override
    public Mat processFrame(Mat input) {
        // Clear contours from the previous frame
        contours.clear();

        // Convert the input frame to HSV color space
        Imgproc.cvtColor(input, hsvFrame, Imgproc.COLOR_RGB2HSV);

        // Detect yellow
        Core.inRange(hsvFrame, lowerYellow, upperYellow, yellowMask);

        // Find contours of the yellow mask
        Imgproc.findContours(yellowMask, contours, new Mat(), Imgproc.RETR_EXTERNAL, Imgproc.CHAIN_APPROX_SIMPLE);

        // Find the largest yellow contour (blob)
        MatOfPoint largestContour = findLargestContour(contours);

        if (largestContour != null) {
            // Draw a red outline around the largest detected object
            Imgproc.drawContours(input, contours, contours.indexOf(largestContour), new Scalar(255, 0, 0), 2);

            // Calculate the centroid of the largest contour
            Moments moments = Imgproc.moments(largestContour);
            cX = moments.get_m10() / moments.get_m00();
            cY = moments.get_m01() / moments.get_m00();


            // Draw a dot at the centroid
            Imgproc.circle(input, new Point(cX, cY), 5, new Scalar(0, 255, 0), -1);

            // Label the centroid coordinates
            String label = "(" + (int) cX + ", " + (int) cY + ")";
            Imgproc.putText(input, label, new Point(cX + 10, cY), Imgproc.FONT_HERSHEY_COMPLEX, 0.5, new Scalar(0, 255, 0), 2);

            // Calculate the width of the bounding box
            width = calculateWidth(largestContour);

            // Display the width next to the label
            String widthLabel = "Width: " + (int) width + " pixels";
            Imgproc.putText(input, widthLabel, new Point(cX + 10, cY + 20), Imgproc.FONT_HERSHEY_SIMPLEX, 0.5, new Scalar(0, 255, 0), 2);
            //Display the Distance
        }
        return input;
    }

    private Mat preprocessFrame(Mat frame) {
        Mat hsvFrame = new Mat();
        Imgproc.cvtColor(frame, hsvFrame, Imgproc.COLOR_BGR2HSV);

        Mat yellowMask = new Mat();
        Core.inRange(hsvFrame, lowerYellow, upperYellow, yellowMask);

        Mat kernel = Imgproc.getStructuringElement(Imgproc.MORPH_RECT, new Size(5, 5));
        Imgproc.morphologyEx(yellowMask, yellowMask, Imgproc.MORPH_OPEN, kernel);
        Imgproc.morphologyEx(yellowMask, yellowMask, Imgproc.MORPH_CLOSE, kernel);

        return yellowMask;
    }

    private MatOfPoint findLargestContour(List<MatOfPoint> contours) {
        double maxArea = 0;
        MatOfPoint largestContour = null;

        for (MatOfPoint contour : contours) {
            double area = Imgproc.contourArea(contour);
            if (area > maxArea) {
                maxArea = area;
                largestContour = contour;
            }
        }

        return largestContour;
    }
    private double calculateWidth(MatOfPoint contour) {
        Rect boundingRect = Imgproc.boundingRect(contour);
        return boundingRect.width;
    }


    public double getcX() {
        return cX;
    }
    public double getcY() {
        return cY;
    }

}
