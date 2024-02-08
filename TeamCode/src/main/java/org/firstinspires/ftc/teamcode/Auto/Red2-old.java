//package org.firstinspires.ftc.teamcode.Auto;
//
//import static org.firstinspires.ftc.teamcode.Auto.Blue2.DRIVE_TRUSS_ANGLE;
//import static org.firstinspires.ftc.teamcode.Hardware.Variables.CVVariables.CAMERA_HEIGHT;
//import static org.firstinspires.ftc.teamcode.Hardware.Variables.CVVariables.CAMERA_WIDTH;
//import static org.firstinspires.ftc.teamcode.Hardware.Variables.CVVariables.higherHB;
//import static org.firstinspires.ftc.teamcode.Hardware.Variables.CVVariables.higherSB;
//import static org.firstinspires.ftc.teamcode.Hardware.Variables.CVVariables.higherVB;
//import static org.firstinspires.ftc.teamcode.Hardware.Variables.CVVariables.lowerHB;
//import static org.firstinspires.ftc.teamcode.Hardware.Variables.CVVariables.*;
//import static org.firstinspires.ftc.teamcode.Hardware.Variables.CVVariables.lowerSB;
//import static org.firstinspires.ftc.teamcode.Hardware.Variables.CVVariables.lowerVB;
//import static org.firstinspires.ftc.teamcode.Hardware.Variables.IntakeArmVariables.INTAKE_PIXEL;
//import static org.firstinspires.ftc.teamcode.Hardware.Variables.IntakeArmVariables.START;
//import static org.firstinspires.ftc.teamcode.Hardware.Variables.IntakeArmVariables.da;
//import static org.firstinspires.ftc.teamcode.Hardware.Variables.IntakeArmVariables.i;
//import static org.firstinspires.ftc.teamcode.Hardware.Variables.IntakeArmVariables.pa;
//import static org.firstinspires.ftc.teamcode.Hardware.Variables.IntakeClawVariables.CLOSE;
//import static org.firstinspires.ftc.teamcode.Hardware.Variables.IntakeClawVariables.RELEASE_TWO;
//import static org.firstinspires.ftc.teamcode.Hardware.Variables.IntakeTimeVariables.INTAKE_AFTER_SLIDES;
//import static org.firstinspires.ftc.teamcode.Hardware.Variables.IntakeTimeVariables.INTAKE_AUTON_RESET;
//import static org.firstinspires.ftc.teamcode.Hardware.Variables.IntakeTimeVariables.INTAKE_CLAW_RESET_TIME;
//import static org.firstinspires.ftc.teamcode.Hardware.Variables.IntakeTimeVariables.INTAKE_CLAW_SERVO_TIME;
//import static org.firstinspires.ftc.teamcode.Hardware.Variables.OuttakeArmVariables.AS_DEPOSIT;
//import static org.firstinspires.ftc.teamcode.Hardware.Variables.OuttakeArmVariables.AS_INIT;
//import static org.firstinspires.ftc.teamcode.Hardware.Variables.OuttakeArmVariables.AUTON_POS;
//import static org.firstinspires.ftc.teamcode.Hardware.Variables.OuttakeArmVariables.DPAD_LEFT;
//import static org.firstinspires.ftc.teamcode.Hardware.Variables.OuttakeArmVariables.LS_DEPOSIT;
//import static org.firstinspires.ftc.teamcode.Hardware.Variables.OuttakeArmVariables.LS_INIT;
//import static org.firstinspires.ftc.teamcode.Hardware.Variables.OuttakeArmVariables.RS_DEPOSIT;
//import static org.firstinspires.ftc.teamcode.Hardware.Variables.OuttakeArmVariables.RS_INIT;
//import static org.firstinspires.ftc.teamcode.Hardware.Variables.OuttakeClawVariables.OUTTAKE_CLOSE;
//import static org.firstinspires.ftc.teamcode.Hardware.Variables.OuttakeClawVariables.RELEASE2;
//import static org.firstinspires.ftc.teamcode.Hardware.Variables.OuttakeTimeVariables.CLAW_MOVE_TIME;
//
//import com.acmerobotics.dashboard.FtcDashboard;
//import com.acmerobotics.dashboard.config.Config;
//import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
//import com.acmerobotics.roadrunner.geometry.Pose2d;
//import com.acmerobotics.roadrunner.geometry.Vector2d;
//import com.acmerobotics.roadrunner.trajectory.Trajectory;
//import com.acmerobotics.roadrunner.util.Angle;
//import com.arcrobotics.ftclib.controller.PIDController;
//import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
//import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
//import com.qualcomm.robotcore.hardware.DcMotorEx;
//import com.qualcomm.robotcore.hardware.Servo;
//import com.qualcomm.robotcore.util.ElapsedTime;
//
//import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
//import org.firstinspires.ftc.teamcode.Hardware.Drivetrain;
//import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
//import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;
//import org.firstinspires.ftc.teamcode.util.ErrorCalc;
//import org.opencv.core.Core;
//import org.opencv.core.Mat;
//import org.opencv.core.MatOfPoint;
//import org.opencv.core.Point;
//import org.opencv.core.Rect;
//import org.opencv.core.Scalar;
//import org.opencv.core.Size;
//import org.opencv.imgproc.Imgproc;
//import org.opencv.imgproc.Moments;
//import org.openftc.easyopencv.OpenCvCamera;
//import org.openftc.easyopencv.OpenCvCameraFactory;
//import org.openftc.easyopencv.OpenCvCameraRotation;
//import org.openftc.easyopencv.OpenCvPipeline;
//
//import java.util.ArrayList;
//import java.util.List;
//
//@Config
//@Autonomous(name = "Red 2", group = "Red")
//public class Red2 extends LinearOpMode {
//
//    double cX;
//    double cY;
//    double width = 0;
//    private OpenCvCamera controlHubCam;
//    private Scalar lowerRed = new Scalar(lowerHR, lowerSR, lowerVR);
//    private Scalar upperRed = new Scalar(higherHR, higherSR, higherVR);
//    String cube_position;
//
//    public static double WAIT_TIME = 3;
//
//    public static double START_X = -37;
//    public static double START_Y = -61.5;
//    public static double START_ANGLE = 90;
//
//    public static double SPIKE_MARK_X_RIGHT = -36;
//    public static double SPIKE_MARK_Y_RIGHT = -32;
//    public static double BACKDROP_X_RIGHT = 46.5;
//    public static double BACKDROP_Y_RIGHT = -30;
//
//    public static double SPIKE_MARK_X_MIDDLE = -38;
//    public static double SPIKE_MARK_Y_MIDDLE = -11;
//    public static double BACKDROP_X_MIDDLE = 46.5;
//    public static double BACKDROP_Y_MIDDLE = -37.5;
//
//    public static double SPIKE_MARK_X_LEFT = -36;
//    public static double SPIKE_MARK_Y_LEFT = -32;
//    public static double BACKDROP_X_LEFT = 46.5;
//    public static double BACKDROP_Y_LEFT = -43;
//
//    public static double SPIKE_MARK_ANGLE_MIDDLE = 270;
//    public static double SPIKE_MARK_ANGLE_LEFT = 180;
//    public static double SPIKE_MARK_ANGLE = 0;
//    public static double BACKDROP_ANGLE = 180;
//
//    public static double DRIVE_TRUSS_X = -37;
//    public static double DRIVE_TRUSS_Y = -10;
//    public static double INIT_BACKDROP_X = 35;
//    public static double INIT_BACKDROP_Y = -10;
//
//    public static double INIT_PARK_X = 45;
//    public static double INIT_PARK_Y = -7;
//    public static double PARK_X = 50;
//    public static double PARK_Y = -7;
//    public static double PARK_ANGLE = 180;
//
//    public static double xyP = 0.6;
//    public static double headingP = 0.85;
//
//    public enum RobotState {
//        START,
//        DRIVE_TO_SPIKE,
//        DROP_SPIKE,
//        RESET_INTAKE,
//        WAIT_RESET_ARM,
//        MOVE_ARM,
//        MOVE_SLIDES,
//        WAIT_SLIDES,
//        DRIVE_TO_BACKDROP,
//        DEPOSIT_PIXEL,
//        PARK,
//        LOCK_PARK
//    }
//    RobotState robot = RobotState.START;
//    private Servo intakeClaw;
//    private DcMotorEx intakeArm;
//    private Servo leftServo;
//    private Servo rightServo;
//    private Servo angledServo;
//    private Servo outtakeClaw;
//    private ElapsedTime timer = new ElapsedTime();
//    private ElapsedTime runtime = new ElapsedTime();
//    private ElapsedTime intakeClawTimer = new ElapsedTime();
//    private ElapsedTime v4bTimer = new ElapsedTime();
//    private ElapsedTime outtakeArmTimer = new ElapsedTime();
//    private PIDController intakeController;
//    public static int moveIntake = 0;
//
//
//
//
//    @Override
//    public void runOpMode() throws InterruptedException {
//        initOpenCV();
//        FtcDashboard.getInstance().startCameraStream(controlHubCam, 30);
//        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
//        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
//        Drivetrain drivetrain = new Drivetrain();
//        drivetrain.init(hardwareMap);
//
//
//        ErrorCalc calculate = new ErrorCalc();
//
//        /** INTAKE ARM PIDF **/
//        intakeClaw = hardwareMap.get(Servo.class, "intakeClaw0");
//        intakeArm = hardwareMap.get(DcMotorEx.class, "intake1");
//        leftServo = hardwareMap.get(Servo.class, "Left3");
//        rightServo = hardwareMap.get(Servo.class, "Right4");
//        angledServo = hardwareMap.get(Servo.class, "Angle2");
//        outtakeClaw = hardwareMap.get(Servo.class, "outtakeClaw1");
//        intakeArm.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
//        intakeArm.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
//        intakeArm.setDirection(DcMotorEx.Direction.REVERSE);
//        intakeController = new PIDController(pa, i, da);
//        moveIntake = 0;
//        v4bTimer.reset();
//        intakeClawTimer.reset();
//
//
//
//        Pose2d startPose = new Pose2d(START_X, START_Y, Math.toRadians(START_ANGLE));
//
//        double spike_mark_x, spike_mark_y, backdrop_x, backdrop_y;
//
//        drive.setPoseEstimate(startPose);
//        while(!opModeIsActive() && !isStopRequested()) {
//            if (cX < 150) {
//                cube_position = "left";
//            }else if (cX < 440) {
//                cube_position = "middle";
//            }else{
//                cube_position = "right";
//            }
//            telemetry.addData("Position", cube_position);
//            telemetry.update();
//            leftServo.setPosition(LS_INIT);
//            rightServo.setPosition(RS_INIT);
//            angledServo.setPosition(AS_INIT);
//            outtakeClaw.setPosition(OUTTAKE_CLOSE);
//            intakeClaw.setPosition(CLOSE);
//        }
//
//        switch (cube_position) {
//            case "left":
//                spike_mark_x = SPIKE_MARK_X_LEFT;
//                spike_mark_y = SPIKE_MARK_Y_LEFT;
//                backdrop_x = BACKDROP_X_LEFT;
//                backdrop_y = BACKDROP_Y_LEFT;
//                SPIKE_MARK_ANGLE = SPIKE_MARK_ANGLE_LEFT;
//                break;
//            case "middle":
//                spike_mark_x = SPIKE_MARK_X_MIDDLE;
//                spike_mark_y = SPIKE_MARK_Y_MIDDLE;
//                backdrop_x = BACKDROP_X_MIDDLE;
//                backdrop_y = BACKDROP_Y_MIDDLE;
//                SPIKE_MARK_ANGLE = SPIKE_MARK_ANGLE_MIDDLE;
//                break;
//            default:
//                spike_mark_x = SPIKE_MARK_X_RIGHT;
//                spike_mark_y = SPIKE_MARK_Y_RIGHT;
//                backdrop_x = BACKDROP_X_RIGHT;
//                backdrop_y = BACKDROP_Y_RIGHT;
//                break;
//        }
//        Trajectory DROP_SPIKE = drive.trajectoryBuilder(startPose)
//                .lineToSplineHeading(new Pose2d(spike_mark_x, spike_mark_y, Math.toRadians(SPIKE_MARK_ANGLE)))
//                .build();
//        TrajectorySequence DRIVE_TRUSS = drive.trajectorySequenceBuilder(DROP_SPIKE.end())
//                .lineToSplineHeading(new Pose2d(DRIVE_TRUSS_X, DRIVE_TRUSS_Y, Math.toRadians(DRIVE_TRUSS_ANGLE)))
//                .waitSeconds(WAIT_TIME)
//                .lineToSplineHeading(new Pose2d(INIT_BACKDROP_X, INIT_BACKDROP_Y, Math.toRadians(DRIVE_TRUSS_ANGLE)))
//                .build();
//        TrajectorySequence DROP_PIXEL = drive.trajectorySequenceBuilder(DRIVE_TRUSS.end())
//                .turn(Math.toRadians(BACKDROP_ANGLE))
//                .lineToSplineHeading(new Pose2d(backdrop_x,backdrop_y,Math.toRadians(BACKDROP_ANGLE)))
//                .build();
//        TrajectorySequence PARK = drive.trajectorySequenceBuilder(DROP_PIXEL.end())
//                .lineToSplineHeading(new Pose2d(INIT_PARK_X,INIT_PARK_Y,Math.toRadians(PARK_ANGLE)))
//                .lineToSplineHeading(new Pose2d(PARK_X,PARK_Y,Math.toRadians(PARK_ANGLE)))
//                .build();
//
//
//        waitForStart();
//
//        if (isStopRequested()) return;
//        Pose2d poseEstimate = drive.getPoseEstimate();
//
//        while (opModeIsActive() && !isStopRequested()) {
//            switch (robot) {
//                case START:
//                    leftServo.setPosition(LS_INIT);
//                    rightServo.setPosition(RS_INIT);
//                    angledServo.setPosition(AS_INIT);
//                    outtakeClaw.setPosition(OUTTAKE_CLOSE);
//                    intakeClaw.setPosition(CLOSE);
//                    robot = RobotState.DRIVE_TO_SPIKE;
//                    break;
//                case DRIVE_TO_SPIKE:
//                    drive.followTrajectory(DROP_SPIKE);
//                    if (!drive.isBusy()) {
//                        v4bTimer.reset();
//                        robot = RobotState.DROP_SPIKE;
//                    }
//                    break;
//                case DROP_SPIKE:
//                    moveIntake = INTAKE_PIXEL;
//                    if (v4bTimer.seconds() >= INTAKE_CLAW_SERVO_TIME) {
//                        intakeClaw.setPosition(RELEASE_TWO);
//                        intakeClawTimer.reset();
//                        robot = RobotState.RESET_INTAKE;
//                    }
//                    break;
//                case RESET_INTAKE:
//                    if (intakeClawTimer.seconds() >= INTAKE_CLAW_RESET_TIME) {
//                        moveIntake = START;
//                        intakeClaw.setPosition(CLOSE);
//                        intakeClawTimer.reset();
//                        robot = RobotState.WAIT_RESET_ARM;
//                    }
//                    break;
//
//                case WAIT_RESET_ARM:
//                    if (intakeClawTimer.seconds() >= INTAKE_AUTON_RESET) {
//                        drive.followTrajectorySequence(DRIVE_TRUSS);
//                        intakeClawTimer.reset();
//                        robot = RobotState.MOVE_ARM;
//                    }
//                    break;
//                case MOVE_ARM:
//                    if (intakeClawTimer.seconds() >= INTAKE_AUTON_RESET) {
//                        leftServo.setPosition(LS_DEPOSIT);
//                        rightServo.setPosition((RS_DEPOSIT));
//                        angledServo.setPosition(AS_DEPOSIT);
//                        intakeClawTimer.reset();
//                        robot = RobotState.MOVE_SLIDES;
//                    }
//                    break;
//                case MOVE_SLIDES:
//                    if (intakeClawTimer.seconds() >= INTAKE_AFTER_SLIDES){
//                        drivetrain.moveSlides(AUTON_POS);
//                        intakeClawTimer.reset();
//                        robot = RobotState.WAIT_SLIDES;
//                    }
//                    break;
//                case WAIT_SLIDES:
//                    if(intakeClawTimer.seconds() >= INTAKE_AFTER_SLIDES){
//                        robot = RobotState.DRIVE_TO_BACKDROP;
//                    }
//                    break;
//                case DRIVE_TO_BACKDROP:
//                    drive.followTrajectorySequence(DROP_PIXEL);
//                    if (!drive.isBusy()) {
//                        robot = RobotState.DEPOSIT_PIXEL;
//                    }
//                    break;
//                case DEPOSIT_PIXEL:
//                    if (!drive.isBusy()) {
//                        outtakeClaw.setPosition(RELEASE2);
//                        outtakeArmTimer.reset();
//                        robot = RobotState.PARK;
//                    }
//                    break;
//                case PARK:
//                    if (outtakeArmTimer.seconds() >= CLAW_MOVE_TIME) {
//                        drivetrain.moveSlides(DPAD_LEFT);
//                        leftServo.setPosition(LS_INIT);
//                        rightServo.setPosition((RS_INIT));
//                        angledServo.setPosition(AS_INIT);
//                        drive.followTrajectorySequence(PARK);
//                        if(!drive.isBusy()){
//                            drive.setPoseEstimate(new Pose2d(0,0,Math.toRadians(0)));
//                            drive.update();
//                            robot = RobotState.LOCK_PARK;
//                        }
//                    }
//                    break;
//                case LOCK_PARK:
//                    lockTo(new Pose2d(0,0,Math.toRadians(0)),drive);
//                    break;
//            }
//            drive.update();
//
//            intakeController.setPID(pa, i, da);
//            int armPos = intakeArm.getCurrentPosition();
//            double intakePID = intakeController.calculate(armPos, moveIntake);
//            double power = intakePID;
//            intakeArm.setPower(power);
//            telemetry.addData("Cube Position", cube_position);
//            telemetry.addData("Robot State: ", robot);
//            telemetry.addData("Intake Position", intakeArm.getCurrentPosition());
//            telemetry.addData("Intake Target", moveIntake);
//            telemetry.addData("finalX", poseEstimate.getX());
//            telemetry.addData("finalY", poseEstimate.getY());
//            telemetry.addData("finalHeading", poseEstimate.getHeading());
//            telemetry.update();
//
//        }
//
//    }
//    public void lockTo(Pose2d targetPos, SampleMecanumDrive drive){
//        Pose2d currPos = drive.getPoseEstimate();
//        Pose2d difference = targetPos.minus(currPos);
//        Vector2d xy = difference.vec().rotated(-currPos.getHeading());
//
//        double heading  = Angle.normDelta(targetPos.getHeading()) - Angle.normDelta(currPos.getHeading());
//        drive.setWeightedDrivePower(new Pose2d(xy.times(xyP), heading * headingP));
//    }
//    private void initOpenCV() {
//
//        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier(
//                "cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
//
//        controlHubCam = OpenCvCameraFactory.getInstance().createWebcam(
//                hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);
//
//        controlHubCam.setPipeline(new RedDetection());
//
//        controlHubCam.openCameraDevice();
//        controlHubCam.startStreaming(CAMERA_WIDTH, CAMERA_HEIGHT, OpenCvCameraRotation.UPRIGHT);
//    }
//
//
//    class RedDetection extends OpenCvPipeline {
//        private Mat hsvFrame = new Mat();
//        private Mat yellowMask = new Mat();
//        private List<MatOfPoint> contours = new ArrayList<>();
//        private Scalar color = new Scalar(255, 0, 0);
//
//        @Override
//        public Mat processFrame(Mat input) {
//            // Clear contours from the previous frame
//            contours.clear();
//
//            // Convert the input frame to HSV color space
//            Imgproc.cvtColor(input, hsvFrame, Imgproc.COLOR_RGB2HSV);
//
//            // Detect yellow
//            Core.inRange(hsvFrame, lowerRed, upperRed, yellowMask);
//
//            // Find contours of the yellow mask
//            Imgproc.findContours(yellowMask, contours, new Mat(), Imgproc.RETR_EXTERNAL, Imgproc.CHAIN_APPROX_SIMPLE);
//
//            // Find the largest yellow contour (blob)
//            MatOfPoint largestContour = findLargestContour(contours);
//
//            if (largestContour != null) {
//                // Draw a red outline around the largest detected object
//                Imgproc.drawContours(input, contours, contours.indexOf(largestContour), new Scalar(255, 0, 0), 2);
//
//                // Calculate the centroid of the largest contour
//                Moments moments = Imgproc.moments(largestContour);
//                cX = moments.get_m10() / moments.get_m00();
//                cY = moments.get_m01() / moments.get_m00();
//
//
//                // Draw a dot at the centroid
//                Imgproc.circle(input, new Point(cX, cY), 5, new Scalar(0, 255, 0), -1);
//
//                // Label the centroid coordinates
//                String label = "(" + (int) cX + ", " + (int) cY + ")";
//                Imgproc.putText(input, label, new Point(cX + 10, cY), Imgproc.FONT_HERSHEY_COMPLEX, 0.5, new Scalar(0, 255, 0), 2);
//
//                // Calculate the width of the bounding box
//                width = calculateWidth(largestContour);
//
//
//
//                // Display the width next to the label
//                String widthLabel = "Width: " + (int) width + " pixels";
//                Imgproc.putText(input, widthLabel, new Point(cX + 10, cY + 20), Imgproc.FONT_HERSHEY_SIMPLEX, 0.5, new Scalar(0, 255, 0), 2);
//                String positionLabel = "Position: " + cube_position;
//                Imgproc.putText(input, positionLabel, new Point(cX + 10, cY + 40), Imgproc.FONT_HERSHEY_SIMPLEX, 0.5, new Scalar(0, 255, 0), 2);
//
//            }
//            return input;
//        }
//
//        private Mat preprocessFrame(Mat frame) {
//            Mat hsvFrame = new Mat();
//            Imgproc.cvtColor(frame, hsvFrame, Imgproc.COLOR_BGR2HSV);
//
//            Mat yellowMask = new Mat();
//            Core.inRange(hsvFrame, lowerRed, upperRed, yellowMask);
//
//            Mat kernel = Imgproc.getStructuringElement(Imgproc.MORPH_RECT, new Size(5, 5));
//            Imgproc.morphologyEx(yellowMask, yellowMask, Imgproc.MORPH_OPEN, kernel);
//            Imgproc.morphologyEx(yellowMask, yellowMask, Imgproc.MORPH_CLOSE, kernel);
//
//            return yellowMask;
//        }
//
//        private MatOfPoint findLargestContour(List<MatOfPoint> contours) {
//            double maxArea = 0;
//            MatOfPoint largestContour = null;
//
//            for (MatOfPoint contour : contours) {
//                double area = Imgproc.contourArea(contour);
//                if (area > maxArea) {
//                    maxArea = area;
//                    largestContour = contour;
//                }
//            }
//
//            return largestContour;
//        }
//        private double calculateWidth(MatOfPoint contour) {
//            Rect boundingRect = Imgproc.boundingRect(contour);
//            return boundingRect.width;
//        }
//    }
//
//}
