//package org.firstinspires.ftc.teamcode.TeleOp;
//
//import static org.firstinspires.ftc.teamcode.Hardware.Variables.AnglePID.AKd;
//import static org.firstinspires.ftc.teamcode.Hardware.Variables.AnglePID.AKi;
//import static org.firstinspires.ftc.teamcode.Hardware.Variables.AnglePID.AKp;
//import static org.firstinspires.ftc.teamcode.Hardware.Variables.AnglePID.VKd;
//import static org.firstinspires.ftc.teamcode.Hardware.Variables.AnglePID.VKi;
//import static org.firstinspires.ftc.teamcode.Hardware.Variables.AnglePID.VKp;
//import static org.firstinspires.ftc.teamcode.Hardware.Variables.CVVariables.CAMERA_HEIGHT;
//import static org.firstinspires.ftc.teamcode.Hardware.Variables.CVVariables.CAMERA_WIDTH;
//import static org.firstinspires.ftc.teamcode.Hardware.Variables.CVVariables.focalLength;
//import static org.firstinspires.ftc.teamcode.Hardware.Variables.CVVariables.higherH;
//import static org.firstinspires.ftc.teamcode.Hardware.Variables.CVVariables.higherS;
//import static org.firstinspires.ftc.teamcode.Hardware.Variables.CVVariables.higherV;
//import static org.firstinspires.ftc.teamcode.Hardware.Variables.CVVariables.lowerH;
//import static org.firstinspires.ftc.teamcode.Hardware.Variables.CVVariables.lowerS;
//import static org.firstinspires.ftc.teamcode.Hardware.Variables.CVVariables.lowerV;
//import static org.firstinspires.ftc.teamcode.Hardware.Variables.CVVariables.objectWidthInRealWorldUnits;
//import static org.firstinspires.ftc.teamcode.Hardware.Variables.CVVariables.widthFocalLength;
//import static org.firstinspires.ftc.teamcode.Hardware.Variables.IntakeArmVariables.HOVER_PIXEL;
//import static org.firstinspires.ftc.teamcode.Hardware.Variables.IntakeArmVariables.INTAKE_PIXEL;
//import static org.firstinspires.ftc.teamcode.Hardware.Variables.IntakeArmVariables.TRANSFER_PIXEL;
//import static org.firstinspires.ftc.teamcode.Hardware.Variables.IntakeArmVariables.d;
//import static org.firstinspires.ftc.teamcode.Hardware.Variables.IntakeArmVariables.f;
//import static org.firstinspires.ftc.teamcode.Hardware.Variables.IntakeArmVariables.i;
//import static org.firstinspires.ftc.teamcode.Hardware.Variables.IntakeArmVariables.maxAcceleration;
//import static org.firstinspires.ftc.teamcode.Hardware.Variables.IntakeArmVariables.maxJerk;
//import static org.firstinspires.ftc.teamcode.Hardware.Variables.IntakeArmVariables.maxVelocity;
//import static org.firstinspires.ftc.teamcode.Hardware.Variables.IntakeArmVariables.p;
//import static org.firstinspires.ftc.teamcode.Hardware.Variables.IntakeClawVariables.CLOSE;
//import static org.firstinspires.ftc.teamcode.Hardware.Variables.IntakeClawVariables.RELEASE_TWO;
//import static org.firstinspires.ftc.teamcode.Hardware.Variables.IntakeTimeVariables.INTAKE_CLAW_SERVO_TIME;
//import static org.firstinspires.ftc.teamcode.Hardware.Variables.IntakeTimeVariables.INTAKE_HOVER_GRAB_TIME;
//import static org.firstinspires.ftc.teamcode.Hardware.Variables.IntakeTimeVariables.INTAKE_RESET_TIME;
//import static org.firstinspires.ftc.teamcode.Hardware.Variables.IntakeTimeVariables.INTAKE_TRANSFER_TIME;
//import static org.firstinspires.ftc.teamcode.Hardware.Variables.OuttakeArmVariables.AS_DEPOSIT;
//import static org.firstinspires.ftc.teamcode.Hardware.Variables.OuttakeArmVariables.AS_GRAB;
//import static org.firstinspires.ftc.teamcode.Hardware.Variables.OuttakeArmVariables.AS_INIT;
//import static org.firstinspires.ftc.teamcode.Hardware.Variables.OuttakeArmVariables.LS_DEPOSIT;
//import static org.firstinspires.ftc.teamcode.Hardware.Variables.OuttakeArmVariables.LS_GRAB;
//import static org.firstinspires.ftc.teamcode.Hardware.Variables.OuttakeArmVariables.LS_GRAB2;
//import static org.firstinspires.ftc.teamcode.Hardware.Variables.OuttakeArmVariables.LS_GRAB3;
//import static org.firstinspires.ftc.teamcode.Hardware.Variables.OuttakeArmVariables.LS_INIT;
//import static org.firstinspires.ftc.teamcode.Hardware.Variables.OuttakeArmVariables.RS_DEPOSIT;
//import static org.firstinspires.ftc.teamcode.Hardware.Variables.OuttakeArmVariables.RS_GRAB;
//import static org.firstinspires.ftc.teamcode.Hardware.Variables.OuttakeArmVariables.RS_GRAB2;
//import static org.firstinspires.ftc.teamcode.Hardware.Variables.OuttakeArmVariables.RS_GRAB3;
//import static org.firstinspires.ftc.teamcode.Hardware.Variables.OuttakeArmVariables.RS_INIT;
//import static org.firstinspires.ftc.teamcode.Hardware.Variables.OuttakeClawVariables.OUTTAKE_CLOSE;
//import static org.firstinspires.ftc.teamcode.Hardware.Variables.OuttakeClawVariables.RELEASE1;
//import static org.firstinspires.ftc.teamcode.Hardware.Variables.OuttakeClawVariables.RELEASE2;
//import static org.firstinspires.ftc.teamcode.Hardware.Variables.OuttakeTimeVariables.CLAW_MOVE_TIME;
//import static org.firstinspires.ftc.teamcode.Hardware.Variables.OuttakeTimeVariables.FLIP_PIXEL_TIME;
//import static org.firstinspires.ftc.teamcode.Hardware.Variables.OuttakeTimeVariables.GRAB_PIXEL_TIME;
//import static org.firstinspires.ftc.teamcode.Hardware.Variables.OuttakeTimeVariables.HOVER_PIXEL_TIME;
//import static org.firstinspires.ftc.teamcode.Hardware.Variables.OuttakeTimeVariables.OUTTAKE_CLAW_SERVO_TIME;
//import static org.firstinspires.ftc.teamcode.Hardware.Variables.OuttakeTimeVariables.WRIST_INIT_TIME;
//
//import com.acmerobotics.dashboard.FtcDashboard;
//import com.acmerobotics.dashboard.config.Config;
//import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
//import com.arcrobotics.ftclib.controller.PIDController;
//import com.qualcomm.hardware.bosch.BNO055IMU;
//import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
//import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
//import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
//import com.qualcomm.robotcore.hardware.DcMotor;
//import com.qualcomm.robotcore.hardware.DcMotorEx;
//import com.qualcomm.robotcore.hardware.IMU;
//import com.qualcomm.robotcore.hardware.Servo;
//import com.qualcomm.robotcore.util.ElapsedTime;
//
//import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
//import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
//import org.firstinspires.ftc.teamcode.Hardware.Drivetrain;
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
////import org.firstinspires.ftc.teamcode.Hardware.Outtake;
//
//@Config
//@TeleOp(name = "TeleOp")
//public class CenterstageTeleOp extends LinearOpMode {
//
//
//    private PIDController intakeController;
//    private PIDController drivetrainController;
//
//
//
//    public enum IntakeState {
//        START,
//        RESET,
//        HOVER_PIXEL,
//        INT_PICKUP_PIXEL,
//        PICK_UP_PIXEL,
//        INT_TRANSFER_PIXEL,
//        TRANSFER_PIXEL
//    }
//    public enum OuttakeState {
//        ARM_INIT,
//        WRIST_INIT,
//        HOVER_GRAB,
//        INIT_ANGLE_GRAB_PIXEL,
//        INIT_MOVE_GRAB_PIXEL,
//        INIT_GRAB_PIXEL,
//        DEPOSIT,
//        OPEN,
//        MOVE_DOWN,
//        GRAB_PIXEL,
//        FLIP_ARM,
//        FLIP_WRIST
//    }
//    OuttakeState outtakeState = OuttakeState.ARM_INIT;
//    IntakeState intakeState = IntakeState.START;
//
//
//
//    public static int moveIntake = 0;
//    private final double ticks_in_degrees  = 193/157.5;
//
//
//    private ElapsedTime runtime = new ElapsedTime();
//    private ElapsedTime intakeClawTimer = new ElapsedTime();
//    private ElapsedTime v4bTimer = new ElapsedTime();
//    private ElapsedTime outtakeArmTimer = new ElapsedTime();
//
//    private DcMotorEx intakeArm;
//    private Servo intakeClaw;
//    private Servo leftServo;
//    private Servo rightServo;
//    private Servo angledServo;
//    private Servo outtakeClaw;
//
//    // OPENCV STUFF
//    ElapsedTime timer = new ElapsedTime();
//    double integralSum = 0;
//    private double lastError = 0;
//    double cX = 0;
//    double cY = 0;
//    double width = 0;
//    private OpenCvCamera controlHubCam;  // Use OpenCvCamera class from FTC SDK
//
//
//    @Override
//    public void runOpMode() throws InterruptedException {
//        Drivetrain drivetrain = new Drivetrain();
//        drivetrain.init(hardwareMap);
////        Outtake outake = new Outtake();
////        outake.init(hardwareMap);
//        intakeArm = hardwareMap.get(DcMotorEx.class, "intake0");
//        intakeClaw = hardwareMap.get(Servo.class, "intakeClaw0");
//        leftServo = hardwareMap.get(Servo.class, "Left3");
//        rightServo = hardwareMap.get(Servo.class, "Right4");
//        angledServo = hardwareMap.get(Servo.class, "Angle2");
//        outtakeClaw = hardwareMap.get(Servo.class, "outtakeClaw1");
//
//        drivetrain.rightSlide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//        drivetrain.leftSlide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//        intakeArm.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
//        intakeArm.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
//        intakeArm.setDirection(DcMotorEx.Direction.REVERSE);
//        intakeArm.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//
//
//
//        intakeController = new PIDController(p,i,d);
//        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
//        moveIntake = 0;
//        v4bTimer.reset();
//        intakeClawTimer.reset();
//        outtakeArmTimer.reset();
//
//
//        // Retrieve the IMU from the hardware map
//        IMU imu = hardwareMap.get(IMU.class, "imu");
//        // Adjust the orientation parameters to match your robot
//        IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(
//                RevHubOrientationOnRobot.LogoFacingDirection.RIGHT,
//                RevHubOrientationOnRobot.UsbFacingDirection.UP));
//        // Without this, the REV Hub's orientation is assumed to be logo up / USB forward
//        imu.initialize(parameters);
//        initOpenCV();
//        FtcDashboard.getInstance().startCameraStream(controlHubCam, 30);
//
//        waitForStart();
//        while(opModeIsActive()) {
//            double y = gamepad1.left_stick_y * multiplier; // Remember, Y stick value is reversed
//            double x = gamepad1.left_stick_x * multiplier;
//            double rx = gamepad1.right_stick_x * multiplier;
//
//            // This button choice was made so that it is hard to hit on accident,
//            // it can be freely changed based on preference.
//            // The equivalent button is start on Xbox-style controllers.
//            if (gamepad1.left_bumper) {
//                imu.resetYaw();
//            }
//            double botHeading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);
//
//            if(gamepad1.start){
//                multiplier = 1;
//            }
//            if(gamepad1.back){
//                multiplier = 0.4;
//            }
//            if(gamepad2.a){
//                intakeClaw.setPosition(CLOSE);
//            }
//
//
//            switch(intakeState) {
//                case START:
//                    if (gamepad1.b) {
//                        intakeClaw.setPosition(CLOSE);
//                        v4bTimer.reset();
//                        intakeState = IntakeState.HOVER_PIXEL;
//                    }
//                    break;
//                case RESET:
//                    if (intakeClawTimer.seconds() >= INTAKE_CLAW_SERVO_TIME) {
//                        intakeClaw.setPosition(CLOSE);
//                        v4bTimer.reset();
//                        intakeState = IntakeState.HOVER_PIXEL;
//                    }
//                    break;
//                case HOVER_PIXEL:
//                    moveIntake = HOVER_PIXEL;
//                    if(v4bTimer.seconds() >= INTAKE_RESET_TIME) {
//                        intakeClaw.setPosition(RELEASE_TWO);
//                        intakeState = IntakeState.INT_PICKUP_PIXEL;
//                    }
//                    break;
//                case INT_PICKUP_PIXEL:
//                    if(gamepad1.a){
//                        moveIntake = INTAKE_PIXEL;
//                        v4bTimer.reset();
//                        intakeState = IntakeState.PICK_UP_PIXEL;
//                    }
//                    break;
//                case PICK_UP_PIXEL:
//                    if(v4bTimer.seconds() >= INTAKE_HOVER_GRAB_TIME){
//                        intakeClaw.setPosition(CLOSE);
//                        intakeClawTimer.reset();
//                        intakeState = IntakeState.INT_TRANSFER_PIXEL;
//                    }
//                    break;
//                case INT_TRANSFER_PIXEL:
//                    if(intakeClawTimer.seconds() >= INTAKE_CLAW_SERVO_TIME){
//                        v4bTimer.reset();
//                        intakeState = IntakeState.TRANSFER_PIXEL;
//                    }
//                    break;
//                case TRANSFER_PIXEL:
//                    moveIntake = TRANSFER_PIXEL;
//                    if(v4bTimer.seconds() >= INTAKE_TRANSFER_TIME){
//                        intakeClaw.setPosition(RELEASE_TWO);
//                        intakeClawTimer.reset();
//                        intakeState = IntakeState.RESET;
//                    }
//                    break;
//                default:
//                    intakeState = IntakeState.START;
//            }
//            switch(outtakeState){
//                case ARM_INIT:
//                    leftServo.setPosition(LS_INIT);
//                    rightServo.setPosition(RS_INIT);
//                    outtakeClaw.setPosition(OUTTAKE_CLOSE);
//                    outtakeArmTimer.reset();
//                    outtakeState = OuttakeState.WRIST_INIT;
//                    break;
//                case WRIST_INIT:
//                    if(outtakeArmTimer.seconds() >= WRIST_INIT_TIME){
//                        angledServo.setPosition(AS_INIT);
//                        outtakeState = OuttakeState.HOVER_GRAB;
//                    }
//                    break;
//                case HOVER_GRAB:
//                    if(gamepad1.x){
//                        leftServo.setPosition(LS_GRAB);
//                        rightServo.setPosition(RS_GRAB);
//                        outtakeArmTimer.reset();
//                        outtakeState = OuttakeState.INIT_ANGLE_GRAB_PIXEL;
//                    }
//                    break;
//                case INIT_ANGLE_GRAB_PIXEL:
//                    if(outtakeArmTimer.seconds() >= HOVER_PIXEL_TIME){
//                        angledServo.setPosition(AS_GRAB);
//                        outtakeArmTimer.reset();
//                        outtakeState = OuttakeState.INIT_MOVE_GRAB_PIXEL;
//                    }
//                    break;
//                case INIT_MOVE_GRAB_PIXEL:
//                    if(outtakeArmTimer.seconds() >= GRAB_PIXEL_TIME){
//                        rightServo.setPosition(RS_GRAB2);
//                        leftServo.setPosition(LS_GRAB2);
//                        outtakeArmTimer.reset();
//                        outtakeState = OuttakeState.OPEN;
//                    }
//                    break;
//                case OPEN:
//                    if(outtakeArmTimer.seconds() >= CLAW_MOVE_TIME){
//                        outtakeClaw.setPosition(RELEASE2);
//                        outtakeArmTimer.reset();
//                        outtakeState = OuttakeState.MOVE_DOWN;
//                    }
//                    break;
//                case MOVE_DOWN:
//                    if(outtakeArmTimer.seconds() >= CLAW_MOVE_TIME){
//                        leftServo.setPosition(LS_GRAB3);
//                        rightServo.setPosition(RS_GRAB3);
//                        outtakeArmTimer.reset();
//                        outtakeState = OuttakeState.GRAB_PIXEL;
//                    }
//                    break;
//                case GRAB_PIXEL:
//                    if(outtakeArmTimer.seconds() >= CLAW_MOVE_TIME){
//                        outtakeClaw.setPosition(OUTTAKE_CLOSE);
//                        outtakeArmTimer.reset();
//                        outtakeState = OuttakeState.FLIP_ARM;
//                    }
//                    break;
//                case FLIP_ARM:
//                    if(outtakeArmTimer.seconds() >= OUTTAKE_CLAW_SERVO_TIME){
//                        leftServo.setPosition(LS_DEPOSIT);
//                        rightServo.setPosition((RS_DEPOSIT));
//                        outtakeArmTimer.reset();
//                        outtakeState = OuttakeState.FLIP_WRIST;
//                    }
//                    break;
//                case FLIP_WRIST:
//                    if(outtakeArmTimer.seconds() >= FLIP_PIXEL_TIME){
//                        angledServo.setPosition(AS_DEPOSIT);
//                        outtakeState = OuttakeState.DEPOSIT;
//                    }
//                    break;
//                case DEPOSIT:
//                    if(gamepad1.dpad_down){
////                        drivetrain.moveSlides();
//                    }
//                    if(gamepad1.right_bumper){
//                        outtakeClaw.setPosition(RELEASE1);
//                    }
//                    if(gamepad1.left_bumper){
//                        outtakeClaw.setPosition(RELEASE2);
//                    }
//                    if(gamepad1.y){
//                        outtakeState = OuttakeState.ARM_INIT;
//                    }
//                    break;
//            }
//
//            intakeController.setPID(p, i, d);
//
//            int armPos = intakeArm.getCurrentPosition();
//            double intakePID = intakeController.calculate(armPos, moveIntake);
//            double ff = Math.cos(Math.toRadians(moveIntake / ticks_in_degrees)) * f;
//
//
//
//
//
//            double power = intakePID + ff;
//            // Apply motion profiling to limit acceleration and jerk
//            double maxDeltaV = maxAcceleration * runtime.seconds();
//            double maxDeltaA = maxJerk * runtime.seconds();
//            power = clamp(power, -maxVelocity, maxVelocity, maxDeltaV, maxDeltaA);
//            if(gamepad1.right_bumper){
//                drivetrain.power(AnglePIDControl(Math.toRadians(0), botHeading));
//                drivetrain.moveRobot(PIDControl(CAMERA_WIDTH/2, cX));
//            }else{
//                drivetrain.moveRobotFieldCentric(y, x,rx,botHeading);
//            }
//            intakeArm.setPower(Math.max(-0.4, Math.min(0.4, power)));
//            telemetry.addData("Pick Up", (getDistance2(width) >= 5.3 && getDistance2(width) <= 5.7));
//            telemetry.addData("X Cord", cX);
//            telemetry.addData("IMU Angle",imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES));
//            telemetry.addData("Left Slide Encoder", drivetrain.leftSlide.getCurrentPosition());
//            telemetry.addData("Right Slide Encoder", drivetrain.rightSlide.getCurrentPosition());
//            telemetry.addData("Current Intake State ", intakeState);
//            telemetry.addData("Current Outtake State", outtakeState);
//            telemetry.addData("claw intake time ", intakeClawTimer.seconds());
//            telemetry.addData("V4b intake time ", v4bTimer.seconds());
//            telemetry.addData("Current Vel ", intakeArm.getVelocity());
//            telemetry.addData("pos ", armPos);
//            telemetry.addData("target ", moveIntake);
//            runtime.reset();
//            telemetry.update();
//        }
//        controlHubCam.stopStreaming();
//    }
//
//    private void initOpenCV() {
//
//        // Create an instance of the camera
//        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier(
//                "cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
//
//        // Use OpenCvCameraFactory class from FTC SDK to create camera instance
//        controlHubCam = OpenCvCameraFactory.getInstance().createWebcam(
//                hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);
//
//        controlHubCam.setPipeline(new YellowBlobDetectionPipeline());
//
//        controlHubCam.openCameraDevice();
//        controlHubCam.startStreaming(CAMERA_WIDTH, CAMERA_HEIGHT, OpenCvCameraRotation.UPRIGHT);
//    }
//    class YellowBlobDetectionPipeline extends OpenCvPipeline {
//        @Override
//        public Mat processFrame(Mat input) {
//            // Preprocess the frame to detect yellow regions
//            Mat yellowMask = preprocessFrame(input);
//
//            // Find contours of the detected yellow regions
//            List<MatOfPoint> contours = new ArrayList<>();
//            Mat hierarchy = new Mat();
//            Imgproc.findContours(yellowMask, contours, hierarchy, Imgproc.RETR_EXTERNAL, Imgproc.CHAIN_APPROX_SIMPLE);
//
//            // Find the largest yellow contour (blob)
//            MatOfPoint largestContour = findLargestContour(contours);
//
//            if (largestContour != null) {
//                // Draw a red outline around the largest detected object
//                Imgproc.drawContours(input, contours, contours.indexOf(largestContour), new Scalar(255, 0, 0), 2);
//                // Calculate the width of the bounding box
//                width = calculateWidth(largestContour);
//
//                // Display the width next to the label
//                String widthLabel = "Width: " + (int) width + " pixels";
//                Imgproc.putText(input, widthLabel, new Point(cX + 10, cY + 20), Imgproc.FONT_HERSHEY_SIMPLEX, 0.5, new Scalar(0, 255, 0), 2);
//                //Display the Distance
//                String distanceLabel = "Distance: " + String.format("%.2f", getDistance2(width)) + " inches";
//                Imgproc.putText(input, distanceLabel, new Point(cX + 10, cY + 60), Imgproc.FONT_HERSHEY_SIMPLEX, 0.5, new Scalar(0, 255, 0), 2);
//                String horiztonalLabel = "Distance: " + String.format("%.2f", getHorizontalDistance(width)) + " inches";
//                Imgproc.putText(input, horiztonalLabel, new Point(cX + 10, cY + 100), Imgproc.FONT_HERSHEY_COMPLEX, 0.5, new Scalar(0, 255,0), 2);
//
//                telemetry.addData("distance label", distanceLabel);
//                telemetry.addData("horizontal label", horiztonalLabel);
//
//
//
//                Imgproc.putText(input, horiztonalLabel, new Point(cX + 10, cY + 60), Imgproc.FONT_HERSHEY_SIMPLEX, 0.5, new Scalar(0, 255, 0), 2);
//                // Calculate the centroid of the largest contour
//                Moments moments = Imgproc.moments(largestContour);
//                cX = moments.get_m10() / moments.get_m00();
//                cY = moments.get_m01() / moments.get_m00();
//
//                // Draw a dot at the centroid
//                String label = "(" + (int) cX + ", " + (int) cY + ")";
//                Imgproc.putText(input, label, new Point(cX + 10, cY), Imgproc.FONT_HERSHEY_COMPLEX, 0.5, new Scalar(0, 255, 0), 2);
//                Imgproc.circle(input, new Point(cX, cY), 5, new Scalar(0, 255, 0), -1);
//
//            }
//
//            return input;
//        }
//
//        private Mat preprocessFrame(Mat frame) {
//            Mat hsvFrame = new Mat();
//            Imgproc.cvtColor(frame, hsvFrame, Imgproc.COLOR_BGR2HSV);
//
//            Scalar lowerYellow = new Scalar(lowerH, lowerS, lowerV);
//            Scalar upperYellow = new Scalar(higherH, higherS, higherV);
//
//
//            Mat yellowMask = new Mat();
//            Core.inRange(hsvFrame, lowerYellow, upperYellow, yellowMask);
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
//
//    }
//    public double AnglePIDControl(double refrence, double state) {
//        double error = angleWrap(refrence - state);
//        telemetry.addData("Error: ", error);
//        integralSum += error * timer.seconds();
//        double derivative = (error - lastError) / (timer.seconds());
//        lastError = error;
//        timer.reset();
//        double output = (error * AKp) + (derivative * AKd) + (integralSum * AKi);
//        return output;
//    }
//    public double PIDControl(double refrence, double state) {
//        double error = refrence - state;
//        telemetry.addData("Error: ", error);
//        integralSum += error * timer.seconds();
//        double derivative = (error - lastError) / (timer.seconds());
//        lastError = error;
//        timer.reset();
//        double output = (error * VKp) + (derivative * VKd) + (integralSum * VKi);
//        return output;
//    }
//    public double angleWrap(double radians) {
//        while (radians > Math.PI) {
//            radians -= 2 * Math.PI;
//        }
//        while (radians < -Math.PI) {
//            radians += 2 * Math.PI;
//        }
//        return radians;
//    }
//    private static double getDistance2(double width){
//        double distance = (objectWidthInRealWorldUnits * focalLength) / width;
//        return distance;
//    }
//    private static double getHorizontalDistance(double width) {
//        double distance = (objectWidthInRealWorldUnits * widthFocalLength) / width;
//        return distance;
//    }
//    private double clamp(double value, double minValue, double maxValue, double maxDeltaValue, double maxDeltaDelta) {
//        if (value > maxValue) {
//            value = maxValue;
//        } else if (value < minValue) {
//            value = minValue;
//        }
//        return value;
//    }
//}
