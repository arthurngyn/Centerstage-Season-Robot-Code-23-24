package org.firstinspires.ftc.teamcode.Auto;

import static org.firstinspires.ftc.teamcode.Hardware.Variables.CVVariables.CAMERA_HEIGHT;
import static org.firstinspires.ftc.teamcode.Hardware.Variables.CVVariables.CAMERA_WIDTH;
import static org.firstinspires.ftc.teamcode.Hardware.Variables.CVVariables.higherHB;
import static org.firstinspires.ftc.teamcode.Hardware.Variables.CVVariables.higherSB;
import static org.firstinspires.ftc.teamcode.Hardware.Variables.CVVariables.higherVB;
import static org.firstinspires.ftc.teamcode.Hardware.Variables.CVVariables.lowerHB;
import static org.firstinspires.ftc.teamcode.Hardware.Variables.CVVariables.lowerSB;
import static org.firstinspires.ftc.teamcode.Hardware.Variables.CVVariables.lowerVB;
import static org.firstinspires.ftc.teamcode.Hardware.Variables.IntakeTimeVariables.ARM_DEPOSIT_TIME;
import static org.firstinspires.ftc.teamcode.Hardware.Variables.IntakeTimeVariables.ARM_MOVE_TIME;
import static org.firstinspires.ftc.teamcode.Hardware.Variables.IntakeTimeVariables.INTAKE_MOTOR_POWER;
import static org.firstinspires.ftc.teamcode.Hardware.Variables.IntakeTimeVariables.INTAKE_MOTOR_TIME;
import static org.firstinspires.ftc.teamcode.Hardware.Variables.IntakeTimeVariables.SLIDES_MOVE_TIME;
import static org.firstinspires.ftc.teamcode.Hardware.Variables.OuttakeArmVariables.AS_DEPOSIT;
import static org.firstinspires.ftc.teamcode.Hardware.Variables.OuttakeArmVariables.AS_INTAKE;
import static org.firstinspires.ftc.teamcode.Hardware.Variables.OuttakeArmVariables.AUTON_POS;
import static org.firstinspires.ftc.teamcode.Hardware.Variables.OuttakeArmVariables.DPAD_LEFT;
import static org.firstinspires.ftc.teamcode.Hardware.Variables.OuttakeArmVariables.LS_DEPOSIT;
import static org.firstinspires.ftc.teamcode.Hardware.Variables.OuttakeArmVariables.LS_INTAKE;
import static org.firstinspires.ftc.teamcode.Hardware.Variables.OuttakeArmVariables.OS_CLOSE;
import static org.firstinspires.ftc.teamcode.Hardware.Variables.OuttakeArmVariables.OS_DEPOSIT_2;
import static org.firstinspires.ftc.teamcode.Hardware.Variables.OuttakeArmVariables.RS_DEPOSIT;
import static org.firstinspires.ftc.teamcode.Hardware.Variables.OuttakeArmVariables.RS_INTAKE;
import static org.firstinspires.ftc.teamcode.Hardware.Variables.OuttakeTimeVariables.FINGER_MOVE;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.acmerobotics.roadrunner.util.Angle;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.Hardware.Drivetrain;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.teamcode.util.ErrorCalc;
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
@Autonomous(name = "Blue 2", group = "Blue")
public class Blue2 extends LinearOpMode {

    double cX;
    double cY;
    double width = 0;
    private OpenCvCamera controlHubCam;
    private Scalar lowerBlue = new Scalar(lowerHB, lowerSB, lowerVB);
    private Scalar upperBlue = new Scalar(higherHB, higherSB, higherVB);
    String cube_position;

    public static double WAIT_TIME = 3;

    public static double START_X = -37;
    public static double START_Y = 61.5;
    public static double START_ANGLE = 270;

    public static double INIT_SPIKE_MARK_X_LEFT = -37;
    public static double INIT_SPIKE_MARK_Y_LEFT = 32.5;
    public static double SPIKE_MARK_X_LEFT = -38;
    public static double SPIKE_MARK_Y_LEFT = 32;
    public static double BACKDROP_X_LEFT = 42;
    public static double BACKDROP_Y_LEFT = 41;

    public static double INIT_SPIKE_MARK_X_MIDDLE = -37;
    public static double INIT_SPIKE_MARK_Y_MIDDLE = 11.5;
    public static double SPIKE_MARK_X_MIDDLE = -38;
    public static double SPIKE_MARK_Y_MIDDLE = 11.5;
    public static double BACKDROP_X_MIDDLE = 42;
    public static double BACKDROP_Y_MIDDLE = 34.25;

    public static double INIT_SPIKE_MARK_X_RIGHT = -37;
    public static double INIT_SPIKE_MARK_Y_RIGHT = 34;
    public static double SPIKE_MARK_X_RIGHT = -40.5;
    public static double SPIKE_MARK_Y_RIGHT = 34;
    public static double BACKDROP_X_RIGHT = 42;
    public static double BACKDROP_Y_RIGHT = 27;

    public static double SPIKE_MARK_ANGLE_MIDDLE = 90;
    public static double SPIKE_MARK_ANGLE_LEFT = 0;
    public static double SPIKE_MARK_ANGLE = 180;
    public static double BACKDROP_ANGLE = 180;
    public static double DRIVE_TRUSS_ANGLE = 180;

    public static double DRIVE_TRUSS_X = -37;
    public static double DRIVE_TRUSS_Y = 10;
    public static double INIT_BACKDROP_X = 35;
    public static double INIT_BACKDROP_Y = 5;

    public static double INIT_PARK_X = 45;
    public static double INIT_PARK_Y = 7;
    public static double PARK_X = 50;
    public static double PARK_Y = 4;
    public static double PARK_ANGLE = 180;

    public static double xyP = 0.6;
    public static double headingP = 0.85;

    public enum RobotState {
        START,
        DRIVE_TO_SPIKE,
        DROP_SPIKE,
        DRIVE_TRUSS,
        MOVE_SLIDES,
        SWING_ARM,
        DRIVE_TO_BACKDROP,
        DEPOSIT_PIXEL,
        INIT_PARK,
        PARK,
        LOCK_PARK
    }
    RobotState robot = RobotState.START;
    private DcMotorEx intakeMotor;
    private Servo leftServo;
    private Servo rightServo;
    private Servo angledServo;
    private Servo outtakeServo;

    private ElapsedTime timer = new ElapsedTime();
    private ElapsedTime runtime = new ElapsedTime();
    private ElapsedTime intakeClawTimer = new ElapsedTime();
    private ElapsedTime v4bTimer = new ElapsedTime();
    private ElapsedTime outtakeArmTimer = new ElapsedTime();
    private PIDController intakeController;
    public static int moveIntake = 0;




    @Override
    public void runOpMode() throws InterruptedException {
        initOpenCV();
        FtcDashboard.getInstance().startCameraStream(controlHubCam, 30);
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        Drivetrain drivetrain = new Drivetrain();
        drivetrain.init(hardwareMap);


        ErrorCalc calculate = new ErrorCalc();

        /** INTAKE ARM PIDF **/
        leftServo = hardwareMap.get(Servo.class, "Left3");
        rightServo = hardwareMap.get(Servo.class, "Right4");
        angledServo = hardwareMap.get(Servo.class, "Angle2");
        outtakeServo = hardwareMap.get(Servo.class, "outtakeClaw1");
        intakeMotor = hardwareMap.get(DcMotorEx.class, "intakeMotor0");
        moveIntake = 0;
        v4bTimer.reset();
        intakeClawTimer.reset();



        Pose2d startPose = new Pose2d(START_X, START_Y, Math.toRadians(START_ANGLE));

        double spike_mark_x, spike_mark_y, backdrop_x, backdrop_y, init_spike_mark_x, init_spike_mark_y;

        drive.setPoseEstimate(startPose);
        while(!opModeIsActive() && !isStopRequested()) {
            if (cX < 150) {
                cube_position = "left";
            }else if (cX < 440) {
                cube_position = "middle";
            }else{
                cube_position = "right";
            }
            telemetry.addData("Position", cube_position);
            telemetry.update();
            leftServo.setPosition(LS_INTAKE);
            rightServo.setPosition(RS_INTAKE);
            angledServo.setPosition(AS_INTAKE);
            outtakeServo.setPosition(OS_CLOSE);
        }

        switch (cube_position) {
            case "left":
                spike_mark_x = SPIKE_MARK_X_LEFT;
                spike_mark_y = SPIKE_MARK_Y_LEFT;
                backdrop_x = BACKDROP_X_LEFT;
                backdrop_y = BACKDROP_Y_LEFT;
                init_spike_mark_x = INIT_SPIKE_MARK_X_LEFT;
                init_spike_mark_y = INIT_SPIKE_MARK_Y_LEFT;
                break;
            case "middle":
                spike_mark_x = SPIKE_MARK_X_MIDDLE;
                spike_mark_y = SPIKE_MARK_Y_MIDDLE;
                backdrop_x = BACKDROP_X_MIDDLE;
                backdrop_y = BACKDROP_Y_MIDDLE;
                init_spike_mark_x = INIT_SPIKE_MARK_X_MIDDLE;
                init_spike_mark_y = INIT_SPIKE_MARK_Y_MIDDLE;
                break;
            default:
                spike_mark_x = SPIKE_MARK_X_RIGHT;
                spike_mark_y = SPIKE_MARK_Y_RIGHT;
                backdrop_x = BACKDROP_X_RIGHT;
                backdrop_y = BACKDROP_Y_RIGHT;
                init_spike_mark_x = INIT_SPIKE_MARK_X_RIGHT;
                init_spike_mark_y = INIT_SPIKE_MARK_Y_RIGHT;
                break;
        }
        TrajectorySequence DROP_SPIKE = drive.trajectorySequenceBuilder(startPose)
                .lineToSplineHeading(new Pose2d(init_spike_mark_x, init_spike_mark_y, Math.toRadians(START_ANGLE)))
                .lineToSplineHeading(new Pose2d(spike_mark_x, spike_mark_y, Math.toRadians(SPIKE_MARK_ANGLE)))
                .build();
        TrajectorySequence DRIVE_TRUSS = drive.trajectorySequenceBuilder(DROP_SPIKE.end())
                .lineToSplineHeading(new Pose2d(DRIVE_TRUSS_X, DRIVE_TRUSS_Y, Math.toRadians(SPIKE_MARK_ANGLE)))
                .lineToSplineHeading(new Pose2d(DRIVE_TRUSS_X, INIT_BACKDROP_Y, Math.toRadians(DRIVE_TRUSS_ANGLE)))
                .waitSeconds(WAIT_TIME)
                .lineToSplineHeading(new Pose2d(INIT_BACKDROP_X, INIT_BACKDROP_Y, Math.toRadians(DRIVE_TRUSS_ANGLE)))
                .build();
        TrajectorySequence DROP_PIXEL = drive.trajectorySequenceBuilder(DRIVE_TRUSS.end())
                .lineToSplineHeading(new Pose2d(backdrop_x, backdrop_y, Math.toRadians(BACKDROP_ANGLE)))
                .build();
        TrajectorySequence INIT_PARK = drive.trajectorySequenceBuilder(DROP_PIXEL.end())
                .lineToSplineHeading(new Pose2d(backdrop_x-5,backdrop_y,Math.toRadians(PARK_ANGLE)))
                .build();
        TrajectorySequence PARK = drive.trajectorySequenceBuilder(INIT_PARK.end())
                .lineToSplineHeading(new Pose2d(INIT_PARK_X,INIT_PARK_Y,Math.toRadians(PARK_ANGLE)))
                .lineToSplineHeading(new Pose2d(PARK_X,PARK_Y,Math.toRadians(PARK_ANGLE)))
                .build();


        waitForStart();

        if (isStopRequested()) return;
        Pose2d poseEstimate = drive.getPoseEstimate();

        while (opModeIsActive() && !isStopRequested()) {
            switch (robot) {
                case START:
                    leftServo.setPosition(LS_INTAKE);
                    rightServo.setPosition(RS_INTAKE);
                    angledServo.setPosition(AS_INTAKE);
                    outtakeServo.setPosition(OS_CLOSE);
                    robot = RobotState.DRIVE_TO_SPIKE;
                    break;
                case DRIVE_TO_SPIKE:
                    drive.followTrajectorySequence(DROP_SPIKE);
                    if (!drive.isBusy()) {
                        timer.reset();
                        robot = RobotState.DROP_SPIKE;
                    }
                    break;
                case DROP_SPIKE:
                    intakeMotor.setPower(INTAKE_MOTOR_POWER);
                    if (timer.seconds() >= INTAKE_MOTOR_TIME) {
                        timer.reset();
                        intakeMotor.setPower(0);
                        robot = RobotState.DRIVE_TRUSS;
                    }
                    break;
                case DRIVE_TRUSS:
                    drive.followTrajectorySequence(DRIVE_TRUSS);
                    if (!drive.isBusy()) {
                        timer.reset();
                        robot = RobotState.MOVE_SLIDES;
                    }
                    break;
                case MOVE_SLIDES:
                    drivetrain.moveSlides(AUTON_POS);
                    if (timer.seconds() >= SLIDES_MOVE_TIME){
                        timer.reset();
                        robot = RobotState.SWING_ARM;
                    }
                    break;
                case SWING_ARM:
                    leftServo.setPosition(LS_DEPOSIT);
                    rightServo.setPosition(RS_DEPOSIT);
                    angledServo.setPosition(AS_DEPOSIT);
                    if(timer.seconds() >= ARM_MOVE_TIME){
                        timer.reset();
                        robot = RobotState.DRIVE_TO_BACKDROP;
                    }
                    break;
                case DRIVE_TO_BACKDROP:
                    drive.followTrajectorySequence(DROP_PIXEL);
                    if (!drive.isBusy()) {
                        timer.reset();
                        robot = RobotState.DEPOSIT_PIXEL;
                    }
                    break;
                case DEPOSIT_PIXEL:
                    outtakeServo.setPosition(OS_DEPOSIT_2);
                    if (timer.seconds() >= FINGER_MOVE){
                        timer.reset();
                        robot = RobotState.INIT_PARK;
                    }
                    break;
                case INIT_PARK:
                    drive.followTrajectorySequence(INIT_PARK);
                    if(!drive.isBusy()){
                        timer.reset();
                        robot = RobotState.PARK;
                    }
                    break;
                case PARK:
                    leftServo.setPosition(LS_INTAKE);
                    rightServo.setPosition((RS_INTAKE));
                    angledServo.setPosition(AS_INTAKE);
                    if (timer.seconds() >= ARM_DEPOSIT_TIME) {
                        drivetrain.moveSlides(DPAD_LEFT);
                        drive.followTrajectorySequence(PARK);
                        if(!drive.isBusy()){
                            drive.setPoseEstimate(new Pose2d(0,0,Math.toRadians(0)));
                            drive.update();
                            robot = RobotState.LOCK_PARK;
                        }
                    }
                    break;
                case LOCK_PARK:
                    lockTo(new Pose2d(0,0,Math.toRadians(0)),drive);
                    break;
            }
            drive.update();

            telemetry.addData("Cube Position", cube_position);
            telemetry.addData("Robot State: ", robot);
            telemetry.addData("Intake Target", moveIntake);
            telemetry.addData("finalX", poseEstimate.getX());
            telemetry.addData("finalY", poseEstimate.getY());
            telemetry.addData("finalHeading", poseEstimate.getHeading());
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
    private void initOpenCV() {

        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier(
                "cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());

        controlHubCam = OpenCvCameraFactory.getInstance().createWebcam(
                hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);

        controlHubCam.setPipeline(new BlueDetection());

        controlHubCam.openCameraDevice();
        controlHubCam.startStreaming(CAMERA_WIDTH, CAMERA_HEIGHT, OpenCvCameraRotation.UPRIGHT);
    }


    class BlueDetection extends OpenCvPipeline {
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
            Core.inRange(hsvFrame, lowerBlue, upperBlue, yellowMask);

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
                String positionLabel = "Position: " + cube_position;
                Imgproc.putText(input, positionLabel, new Point(cX + 10, cY + 40), Imgproc.FONT_HERSHEY_SIMPLEX, 0.5, new Scalar(0, 255, 0), 2);

            }
            return input;
        }

        private Mat preprocessFrame(Mat frame) {
            Mat hsvFrame = new Mat();
            Imgproc.cvtColor(frame, hsvFrame, Imgproc.COLOR_BGR2HSV);

            Mat yellowMask = new Mat();
            Core.inRange(hsvFrame, lowerBlue, upperBlue, yellowMask);

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
