package org.firstinspires.ftc.teamcode.TeleOp;

import static org.firstinspires.ftc.teamcode.Hardware.Variables.CVVariables.CAMERA_HEIGHT;
import static org.firstinspires.ftc.teamcode.Hardware.Variables.CVVariables.CAMERA_WIDTH;
import static org.firstinspires.ftc.teamcode.Hardware.Variables.CVVariables.focalLength;
import static org.firstinspires.ftc.teamcode.Hardware.Variables.CVVariables.higherHY;
import static org.firstinspires.ftc.teamcode.Hardware.Variables.CVVariables.higherSY;
import static org.firstinspires.ftc.teamcode.Hardware.Variables.CVVariables.higherVY;
import static org.firstinspires.ftc.teamcode.Hardware.Variables.CVVariables.lowerHY;
import static org.firstinspires.ftc.teamcode.Hardware.Variables.CVVariables.lowerSY;
import static org.firstinspires.ftc.teamcode.Hardware.Variables.CVVariables.lowerVY;
import static org.firstinspires.ftc.teamcode.Hardware.Variables.CVVariables.objectWidthInRealWorldUnits;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.util.Angle;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
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
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvPipeline;

import java.util.ArrayList;
import java.util.List;

@Config
@TeleOp(name = "Position Lock Test")
public class positionLock extends LinearOpMode {
    double cX = 0;
    double cY = 0;
    double width = 0;
    public static double x = 0;
    public static double y = 0;
    private OpenCvCamera controlHubCam;
    private Scalar lowerYellow = new Scalar(lowerHY, lowerSY, lowerVY);
    private Scalar upperYellow = new Scalar(higherHY, higherSY, higherVY);

    public static double xyP = 0.3;
    public static double headingP = 0.8;

    public static double X_OFFSET = 0.5;
    public static double Y_OFFSET = -4;

    @Override
    public void runOpMode() throws InterruptedException {
        initOpenCV();
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        FtcDashboard.getInstance().startCameraStream(controlHubCam, 30);


        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);



        waitForStart();


        while (opModeIsActive()){

            if(gamepad1.a){
                Pose2d startPose = new Pose2d(0, 0, Math.toRadians(90));
                drive.setPoseEstimate(startPose);
                 y = getVerticalDistance(width);
                 x = getHorizontalDistance(cX,width);

            }
            if(gamepad1.right_bumper){
                lockTo(new Pose2d(-x + X_OFFSET,y + Y_OFFSET,Math.toRadians(90)), drive);
            }
            drive.update();
            telemetry.addData("X: ", x);
            telemetry.addData("Y: ", y);
            telemetry.addData("Vertical Target", getVerticalDistance(width));
            telemetry.addData("Horz Target", getHorizontalDistance(cX,width));
            telemetry.update();
        }



    }
    public void lockTo(Pose2d targetPos, SampleMecanumDrive drive){
        Pose2d currPos = drive.getPoseEstimate();
        Pose2d difference = targetPos.minus(currPos);
        Vector2d xy = difference.vec().rotated(-currPos.getHeading());

        double heading  = Angle.normDelta(targetPos.getHeading()) - Angle.normDelta(currPos.getHeading());
        drive.setWeightedDrivePower(new Pose2d(xy.times(xyP), heading * headingP));
    }


    private double getVerticalDistance(double width){
        double distance = (objectWidthInRealWorldUnits * focalLength) / width;
        return distance;
    }
    private double getHorizontalDistance(double cX, double width) {
        double distancePX = CAMERA_WIDTH / 2 - cX;
        return objectWidthInRealWorldUnits * distancePX / width;
    }
    private void initOpenCV() {

        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier(
                "cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());

        controlHubCam = OpenCvCameraFactory.getInstance().createWebcam(
                hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);

        controlHubCam.setPipeline(new YellowBlobDetectionPipeline());

        controlHubCam.openCameraDevice();
        controlHubCam.startStreaming(CAMERA_WIDTH, CAMERA_HEIGHT, OpenCvCameraRotation.UPRIGHT);
    }
    class YellowBlobDetectionPipeline extends OpenCvPipeline {
        private Mat hsvFrame = new Mat();
        private Mat yellowMask = new Mat();
        private List<MatOfPoint> contours = new ArrayList<>();
        private Scalar color = new Scalar(255, 0, 0);

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
                String distanceLabel = "V: " + String.format("%.2f", getVerticalDistance(width)) + " inches";
                Imgproc.putText(input, distanceLabel, new Point(cX + 10, cY + 40), Imgproc.FONT_HERSHEY_SIMPLEX, 0.5, new Scalar(0, 255, 0), 2);
                String horiztonalLabel = "H: " + String.format("%.2f", getHorizontalDistance(cX, width)) + " inches";
                Imgproc.putText(input, horiztonalLabel, new Point(cX + 10, cY + 60), Imgproc.FONT_HERSHEY_COMPLEX, 0.5, new Scalar(0, 255, 0), 2);
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
    }
}
