package org.firstinspires.ftc.teamcode.TeleOp;

import static org.firstinspires.ftc.teamcode.Hardware.Variables.AnglePID.AKd;
import static org.firstinspires.ftc.teamcode.Hardware.Variables.AnglePID.AKi;
import static org.firstinspires.ftc.teamcode.Hardware.Variables.AnglePID.AKp;
import static org.firstinspires.ftc.teamcode.Hardware.Variables.AnglePID.ANGLE_ERROR;
import static org.firstinspires.ftc.teamcode.Hardware.Variables.AnglePID.HKd;
import static org.firstinspires.ftc.teamcode.Hardware.Variables.AnglePID.HKi;
import static org.firstinspires.ftc.teamcode.Hardware.Variables.AnglePID.HKp;
import static org.firstinspires.ftc.teamcode.Hardware.Variables.AnglePID.HORZ_ERROR;
import static org.firstinspires.ftc.teamcode.Hardware.Variables.AnglePID.VKd;
import static org.firstinspires.ftc.teamcode.Hardware.Variables.AnglePID.VKi;
import static org.firstinspires.ftc.teamcode.Hardware.Variables.AnglePID.VKp;
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
import static org.firstinspires.ftc.teamcode.Hardware.Variables.IntakeTimeVariables.INTAKE_RESET_TIME;
import static org.firstinspires.ftc.teamcode.Hardware.Variables.IntakeTimeVariables.INTAKE_TRANSFER_TIME;
import static org.firstinspires.ftc.teamcode.Hardware.Variables.OuttakeArmVariables.AS_DEPOSIT;
import static org.firstinspires.ftc.teamcode.Hardware.Variables.OuttakeArmVariables.AS_INTAKE;
import static org.firstinspires.ftc.teamcode.Hardware.Variables.OuttakeArmVariables.AS_TILT;
import static org.firstinspires.ftc.teamcode.Hardware.Variables.OuttakeArmVariables.DPAD_DOWN;
import static org.firstinspires.ftc.teamcode.Hardware.Variables.OuttakeArmVariables.DPAD_LEFT;
import static org.firstinspires.ftc.teamcode.Hardware.Variables.OuttakeArmVariables.DPAD_RIGHT;
import static org.firstinspires.ftc.teamcode.Hardware.Variables.OuttakeArmVariables.DPAD_UP;
import static org.firstinspires.ftc.teamcode.Hardware.Variables.OuttakeArmVariables.LS_DEPOSIT;
import static org.firstinspires.ftc.teamcode.Hardware.Variables.OuttakeArmVariables.LS_INTAKE;
import static org.firstinspires.ftc.teamcode.Hardware.Variables.OuttakeArmVariables.OS_CLOSE;
import static org.firstinspires.ftc.teamcode.Hardware.Variables.OuttakeArmVariables.OS_DEPOSIT_1;
import static org.firstinspires.ftc.teamcode.Hardware.Variables.OuttakeArmVariables.OS_DEPOSIT_2;
import static org.firstinspires.ftc.teamcode.Hardware.Variables.OuttakeArmVariables.OS_INTAKE;
import static org.firstinspires.ftc.teamcode.Hardware.Variables.OuttakeArmVariables.RS_DEPOSIT;
import static org.firstinspires.ftc.teamcode.Hardware.Variables.OuttakeArmVariables.RS_INTAKE;
import static org.firstinspires.ftc.teamcode.Hardware.Variables.OuttakeTimeVariables.FINGER_MOVE;
import static org.firstinspires.ftc.teamcode.Hardware.Variables.OuttakeTimeVariables.RESET_ARM_TIMER;
import static org.firstinspires.ftc.teamcode.Hardware.Variables.OuttakeTimeVariables.SLIDES_DOWN_TIMER;
import static org.firstinspires.ftc.teamcode.Hardware.Variables.OuttakeTimeVariables.SWING_ARM;
import static org.firstinspires.ftc.teamcode.Hardware.Variables.OuttakeTimeVariables.TILT_BUCKET;
import static org.firstinspires.ftc.teamcode.Hardware.Variables.ShooterVariables.INIT_SHOOTER;
import static org.firstinspires.ftc.teamcode.Hardware.Variables.ShooterVariables.SHOOT;

import static java.lang.Math.PI;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.Hardware.Drivetrain;
import org.firstinspires.ftc.teamcode.Hardware.KalmanFilter;
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
@TeleOp(name = "BLUE TELEOP")
public class BlueTeleOP extends LinearOpMode {
    private static double multiplier;
    public static double fast = 1;
    public static double slow = 0.3;


    double cX = 0;
    double cY = 0;
    double width = 0;
    private OpenCvCamera controlHubCam;
    private Scalar lowerYellow = new Scalar(lowerHY, lowerSY, lowerVY);
    private Scalar upperYellow = new Scalar(higherHY, higherSY, higherVY);




    private double lastError = 0;
    double integralSum = 0;

    private ElapsedTime timer = new ElapsedTime();


    private double horizontalFieldOfView = Math.toRadians(40);  // Example: 60 degrees

//    private BNO055IMU angleimu;
    private DcMotorEx intakeMotor;
    private Servo leftServo;
    private Servo rightServo;
    private Servo angledServo;
    private Servo outtakeServo;

    private Servo planeServo;
    private double slidesMultiplier = 0.45;

    public enum OuttakeState {
        ARM_INIT,
        TILT_BUCKET,
        CLOSE_FINGER,
        SWING_ARM,
        MOVE_SLIDES,
        RESET_ARM,
        SLIDES_DOWN,



    }
    OuttakeState outtakeState = OuttakeState.ARM_INIT;





    @Override
    public void runOpMode() throws InterruptedException {
        initOpenCV();
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        FtcDashboard.getInstance().startCameraStream(controlHubCam, 30);
        Drivetrain drivetrain = new Drivetrain();
        drivetrain.init(hardwareMap);

        intakeMotor = hardwareMap.get(DcMotorEx.class, "intakeMotor0");
        leftServo = hardwareMap.get(Servo.class, "Left3");
        rightServo = hardwareMap.get(Servo.class, "Right4");
        angledServo = hardwareMap.get(Servo.class, "Angle2");
        outtakeServo = hardwareMap.get(Servo.class, "outtakeClaw1");
        planeServo = hardwareMap.get(Servo.class, "Plane0");

        // Retrieve the IMU from the hardware map
        IMU imu = hardwareMap.get(IMU.class, "imu");
        // Adjust the orientation parameters to match your robot
        IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.RIGHT,
                RevHubOrientationOnRobot.UsbFacingDirection.UP));
        // Without this, the REV Hub's orientation is assumed to be logo up / USB forward
        imu.initialize(parameters);


        drivetrain.leftSlide.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        drivetrain.rightSlide.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);

        drivetrain.rightSlide.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        drivetrain.leftSlide.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);



        planeServo.setPosition(INIT_SHOOTER);

        waitForStart();
        while(opModeIsActive()){
            /** FIELD CENTRIC MOVEMENT **/
            double y = gamepad1.left_stick_y * multiplier; // Remember, Y stick value is reversed
            double x = gamepad1.left_stick_x * multiplier;
            double rx = gamepad1.right_stick_x * multiplier;

            /** RESETTING IMU **/
            if(gamepad1.dpad_left){
                imu.resetYaw();
            }
            /** Changing Multipler Speeds **/
            //FAST
            if(gamepad1.back){
                multiplier = slow;
            }
            //SLOW
            if(gamepad1.start) {
                multiplier = fast;
            }

            // plane shooter
            if (gamepad1.x) {
                planeServo.setPosition(SHOOT);
            }

            if(gamepad2.back){
                drivetrain.leftSlide.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
                drivetrain.rightSlide.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);

                drivetrain.rightSlide.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
                drivetrain.leftSlide.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
            }
            /** Intake Motor SPINNING YI INGOISHIFe  ***/
            intakeMotor.setPower(gamepad1.right_trigger);
            while(gamepad1.right_trigger == 0 && gamepad1.left_trigger > 0){
                intakeMotor.setPower(-gamepad1.left_trigger);
            }

            /** OUTTAKE STATE MACHINE THINGY **/

            switch(outtakeState){
                case ARM_INIT:
                    leftServo.setPosition(LS_INTAKE);
                    rightServo.setPosition(RS_INTAKE);
                    angledServo.setPosition(AS_INTAKE);
                    outtakeServo.setPosition(OS_INTAKE);
                    if(gamepad2.x){
                        timer.reset();
                        outtakeState = OuttakeState.TILT_BUCKET;
                    }
                    break;
                case TILT_BUCKET:
                    drivetrain.moveSlides(DPAD_DOWN);
                    angledServo.setPosition(AS_TILT);
                    if(timer.seconds()>= TILT_BUCKET){
                        timer.reset();
                        outtakeState = OuttakeState.CLOSE_FINGER;
                    }
                    break;
                case CLOSE_FINGER:
                    outtakeServo.setPosition(OS_CLOSE);
                    if(timer.seconds() >= FINGER_MOVE){
                        timer.reset();
                        outtakeState = OuttakeState.SWING_ARM;
                    }
                    break;
                case SWING_ARM:
                    leftServo.setPosition(LS_DEPOSIT);
                    rightServo.setPosition(RS_DEPOSIT);
                    angledServo.setPosition(AS_DEPOSIT);
                    if(timer.seconds() >= SWING_ARM){
                        timer.reset();
                        outtakeState = OuttakeState.MOVE_SLIDES;
                    }
                    break;
                case MOVE_SLIDES:
                    if (gamepad2.dpad_down) {
                        drivetrain.moveSlides(DPAD_DOWN);
                    }
                    if (gamepad2.dpad_up) { //
                        drivetrain.moveSlides(DPAD_UP);
                    }
                    if (gamepad2.dpad_right) {
                        drivetrain.moveSlides(DPAD_RIGHT);
                    }
                    if (gamepad2.dpad_left) {
                        drivetrain.moveSlides(DPAD_LEFT); //initialized at 0
                    }
                    if (gamepad2.right_bumper) {
                        outtakeServo.setPosition(OS_DEPOSIT_2);
                    }
                    if (gamepad2.left_bumper) {
                        outtakeServo.setPosition(OS_DEPOSIT_1);
                    }
                    if (gamepad2.y) {
                        timer.reset();
                        outtakeState = OuttakeState.RESET_ARM;
                    }
                    if (gamepad2.a) {
                        drivetrain.powerSlides(gamepad2.left_stick_y * slidesMultiplier);
                    } else if (drivetrain.getCurrentSlidePower() != 0) {
                        drivetrain.powerSlides(0);
                    }
                    break;
                case RESET_ARM:
                    leftServo.setPosition(LS_INTAKE);
                    rightServo.setPosition(RS_INTAKE);
                    angledServo.setPosition(AS_INTAKE);
                    outtakeServo.setPosition(OS_INTAKE);
                    if(timer.seconds() >= RESET_ARM_TIMER){
                        timer.reset();
                        outtakeState = OuttakeState.SLIDES_DOWN;
                    }
                    break;
                case SLIDES_DOWN:
                    drivetrain.moveSlides(DPAD_LEFT);
                    if(timer.seconds() >= SLIDES_DOWN_TIMER){
                        timer.reset();
                        outtakeState = OuttakeState.ARM_INIT;
                    }
                    break;
            }
            /**
             * Step 1: MOVE SLIDES UP TILT OUTTAKE BUCKET BACK
             * Step 2: CLOSE THE FINGER
             * Step 3: Swing out arm and tilt angle
             * Step 4: Add slides to move up higher and release 1 and release 2
             * Step 5: Reset
             */




            /** MOVEMENT FOR FIELD CENTRIC AND AUTO ALIGN PIXEL **/
            double robotTheta = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);
            drivetrain.moveRobotFieldCentric(y, x,rx,robotTheta);


            telemetry.addData("Vertical distance", getVerticalDistance(width));
            telemetry.addData("Horizontal distance", getHorizontalDistance(cX, width));
            telemetry.addData("Width", width);
            telemetry.addData("bot heading", Math.toDegrees(robotTheta));
            telemetry.addData("Left Slide Position", drivetrain.leftSlide.getCurrentPosition());
            telemetry.addData("Right Slide Position", drivetrain.rightSlide.getCurrentPosition());
            telemetry.update();
            }
        controlHubCam.stopStreaming();
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

    public double AnglePIDControl(double refrence, double state) {
        double error = angleWrap(refrence - state);
        telemetry.addData("Error: ", error);
        integralSum += error * timer.seconds();
        double derivative = (error - lastError) / (timer.seconds());
        lastError = error;
        timer.reset();
        double output = (error * AKp) + (derivative * AKd) + (integralSum * AKi);
        return output;
    }
    public double HorzPIDControl(double refrence, double state) {
        double error = refrence - state;
        telemetry.addData("Error: ", error);
        integralSum += error * timer.seconds();
        double derivative = (error - lastError) / (timer.seconds());
        lastError = error;
        timer.reset();
        double output = (error * HKp) + (derivative * HKd) + (integralSum * HKi);
        return output;
    }


    public double angleWrap(double radians) {
        while (radians > PI) {
            radians -= 2 * PI;
        }
        while (radians < -PI) {
            radians += 2 * PI;
        }
        return radians;
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
            } else {
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
    private double clamp(double value, double minValue, double maxValue, double maxDeltaValue, double maxDeltaDelta) {
        if (value > maxValue) {
            value = maxValue;
        } else if (value < minValue) {
            value = minValue;
        }
        return value;
    }
}
