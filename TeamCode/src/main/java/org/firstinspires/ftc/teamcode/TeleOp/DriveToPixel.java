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
import static org.firstinspires.ftc.teamcode.Hardware.Variables.CVVariables.higherH;
import static org.firstinspires.ftc.teamcode.Hardware.Variables.CVVariables.higherS;
import static org.firstinspires.ftc.teamcode.Hardware.Variables.CVVariables.higherV;
import static org.firstinspires.ftc.teamcode.Hardware.Variables.CVVariables.lowerH;
import static org.firstinspires.ftc.teamcode.Hardware.Variables.CVVariables.lowerS;
import static org.firstinspires.ftc.teamcode.Hardware.Variables.CVVariables.lowerV;
import static org.firstinspires.ftc.teamcode.Hardware.Variables.CVVariables.objectWidthInRealWorldUnits;
import static org.firstinspires.ftc.teamcode.Hardware.Variables.CVVariables.widthFocalLength;
import static org.firstinspires.ftc.teamcode.Hardware.Variables.IntakeArmVariables.AVOID_PIXEL;
import static org.firstinspires.ftc.teamcode.Hardware.Variables.IntakeArmVariables.GRAB_DISTANCE;
import static org.firstinspires.ftc.teamcode.Hardware.Variables.IntakeArmVariables.HOVER_PIXEL;
import static org.firstinspires.ftc.teamcode.Hardware.Variables.IntakeArmVariables.INTAKE_PIXEL;
import static org.firstinspires.ftc.teamcode.Hardware.Variables.IntakeArmVariables.TRANSFER_PIXEL;
import static org.firstinspires.ftc.teamcode.Hardware.Variables.IntakeArmVariables.d;
import static org.firstinspires.ftc.teamcode.Hardware.Variables.IntakeArmVariables.f;
import static org.firstinspires.ftc.teamcode.Hardware.Variables.IntakeArmVariables.i;
import static org.firstinspires.ftc.teamcode.Hardware.Variables.IntakeArmVariables.maxAcceleration;
import static org.firstinspires.ftc.teamcode.Hardware.Variables.IntakeArmVariables.maxJerk;
import static org.firstinspires.ftc.teamcode.Hardware.Variables.IntakeArmVariables.maxVelocity;
import static org.firstinspires.ftc.teamcode.Hardware.Variables.IntakeArmVariables.p;
import static org.firstinspires.ftc.teamcode.Hardware.Variables.IntakeClawVariables.CLOSE;
import static org.firstinspires.ftc.teamcode.Hardware.Variables.IntakeClawVariables.RELEASE_TWO;
import static org.firstinspires.ftc.teamcode.Hardware.Variables.IntakeTimeVariables.INTAKE_CLAW_RESET_TIME;
import static org.firstinspires.ftc.teamcode.Hardware.Variables.IntakeTimeVariables.INTAKE_CLAW_SERVO_TIME;
import static org.firstinspires.ftc.teamcode.Hardware.Variables.IntakeTimeVariables.INTAKE_HOVER_GRAB_TIME;
import static org.firstinspires.ftc.teamcode.Hardware.Variables.IntakeTimeVariables.INTAKE_RESET_TIME;
import static org.firstinspires.ftc.teamcode.Hardware.Variables.IntakeTimeVariables.INTAKE_TRANSFER_TIME;
import static org.firstinspires.ftc.teamcode.Hardware.Variables.OuttakeArmVariables.AS_DEPOSIT;
import static org.firstinspires.ftc.teamcode.Hardware.Variables.OuttakeArmVariables.AS_GRAB;
import static org.firstinspires.ftc.teamcode.Hardware.Variables.OuttakeArmVariables.AS_INIT;
import static org.firstinspires.ftc.teamcode.Hardware.Variables.OuttakeArmVariables.DPAD_DOWN;
import static org.firstinspires.ftc.teamcode.Hardware.Variables.OuttakeArmVariables.DPAD_LEFT;
import static org.firstinspires.ftc.teamcode.Hardware.Variables.OuttakeArmVariables.DPAD_RIGHT;
import static org.firstinspires.ftc.teamcode.Hardware.Variables.OuttakeArmVariables.DPAD_UP;
import static org.firstinspires.ftc.teamcode.Hardware.Variables.OuttakeArmVariables.LS_DEPOSIT;
import static org.firstinspires.ftc.teamcode.Hardware.Variables.OuttakeArmVariables.LS_GRAB;
import static org.firstinspires.ftc.teamcode.Hardware.Variables.OuttakeArmVariables.LS_GRAB2;
import static org.firstinspires.ftc.teamcode.Hardware.Variables.OuttakeArmVariables.LS_GRAB3;
import static org.firstinspires.ftc.teamcode.Hardware.Variables.OuttakeArmVariables.LS_INIT;
import static org.firstinspires.ftc.teamcode.Hardware.Variables.OuttakeArmVariables.RS_DEPOSIT;
import static org.firstinspires.ftc.teamcode.Hardware.Variables.OuttakeArmVariables.RS_GRAB;
import static org.firstinspires.ftc.teamcode.Hardware.Variables.OuttakeArmVariables.RS_GRAB2;
import static org.firstinspires.ftc.teamcode.Hardware.Variables.OuttakeArmVariables.RS_GRAB3;
import static org.firstinspires.ftc.teamcode.Hardware.Variables.OuttakeArmVariables.RS_INIT;
import static org.firstinspires.ftc.teamcode.Hardware.Variables.OuttakeClawVariables.OUTTAKE_CLOSE;
import static org.firstinspires.ftc.teamcode.Hardware.Variables.OuttakeClawVariables.RELEASE1;
import static org.firstinspires.ftc.teamcode.Hardware.Variables.OuttakeClawVariables.RELEASE2;
import static org.firstinspires.ftc.teamcode.Hardware.Variables.OuttakeTimeVariables.CLAW_MOVE_TIME;
import static org.firstinspires.ftc.teamcode.Hardware.Variables.OuttakeTimeVariables.FLIP_PIXEL_TIME;
import static org.firstinspires.ftc.teamcode.Hardware.Variables.OuttakeTimeVariables.GRAB_PIXEL_TIME;
import static org.firstinspires.ftc.teamcode.Hardware.Variables.OuttakeTimeVariables.HOVER_PIXEL_TIME;
import static org.firstinspires.ftc.teamcode.Hardware.Variables.OuttakeTimeVariables.OUTTAKE_CLAW_SERVO_TIME;
import static org.firstinspires.ftc.teamcode.Hardware.Variables.OuttakeTimeVariables.WRIST_INIT_TIME;

import static java.lang.Math.PI;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.rev.Rev2mDistanceSensor;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
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
@TeleOp(name = "Drive to Pixel")
public class DriveToPixel extends LinearOpMode {
    private static double multiplier;
    public static double fast = 1;
    public static double slow = 0.3;


    double cX = 0;
    double cY = 0;
    double width = 0;
    private OpenCvCamera controlHubCam;
    private Scalar lowerYellow = new Scalar(lowerH, lowerS, lowerV);
    private Scalar upperYellow = new Scalar(higherH, higherS, higherV);

    private PIDController distanceController;
    private PIDController horizontalController;
    private PIDController intakeController;


    double forwardPower;

    private double lastError = 0;
    double integralSum = 0;

    private ElapsedTime timer = new ElapsedTime();
    private ElapsedTime runtime = new ElapsedTime();
    private ElapsedTime intakeClawTimer = new ElapsedTime();
    private ElapsedTime v4bTimer = new ElapsedTime();
    private ElapsedTime outtakeArmTimer = new ElapsedTime();


    private double horizontalFieldOfView = Math.toRadians(40);  // Example: 60 degrees
    private boolean detectsObjects = false;

//    private BNO055IMU angleimu;
    private DistanceSensor sensorDistance;
    private Servo intakeClaw;
    private DcMotorEx intakeArm;
    private Servo leftServo;
    private Servo rightServo;
    private Servo angledServo;
    private Servo outtakeClaw;

    public static int moveIntake = 0;
    private final double ticks_in_degrees  = 193/157.5;

    private double slidesMultiplier = 0.45;

    public enum IntakeState {
        START,
        RESET,
        HOVER_PIXEL,
        INT_PICKUP_PIXEL_ONE,
        INT_PICKUP_PIXEL,
        PICK_UP_PIXEL,
        INT_TRANSFER_PIXEL,
        TRANSFER_PIXEL,
        ARM_MOVE_UP
    }
    IntakeState intakeState = IntakeState.START;

    public enum OuttakeState {
        ARM_INIT,
        WRIST_INIT,
        HOVER_GRAB,
        INIT_ANGLE_GRAB_PIXEL,
        INIT_MOVE_GRAB_PIXEL,
        INIT_GRAB_PIXEL,
        DEPOSIT,
        OPEN,
        MOVE_DOWN,
        GRAB_PIXEL,
        FLIP_ARM,
        FLIP_WRIST
    }
    OuttakeState outtakeState = OuttakeState.ARM_INIT;





    @Override
    public void runOpMode() throws InterruptedException {
        initOpenCV();
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        FtcDashboard.getInstance().startCameraStream(controlHubCam, 30);
        distanceController = new PIDController(VKp,VKi,VKd);
        horizontalController = new PIDController(HKp,HKi,HKd);
        Drivetrain drivetrain = new Drivetrain();
        drivetrain.init(hardwareMap);

//        angleimu = hardwareMap.get(BNO055IMU.class, "imu");
//        BNO055IMU.Parameters angleParameters = new BNO055IMU.Parameters();
//        angleParameters.mode = BNO055IMU.SensorMode.IMU;
//        angleParameters.angleUnit = BNO055IMU.AngleUnit.RADIANS;
//        angleimu.initialize(angleParameters);
        sensorDistance = hardwareMap.get(DistanceSensor.class, "distance0");
        intakeClaw = hardwareMap.get(Servo.class, "intakeClaw0");
        leftServo = hardwareMap.get(Servo.class, "Left3");
        rightServo = hardwareMap.get(Servo.class, "Right4");
        angledServo = hardwareMap.get(Servo.class, "Angle2");
        outtakeClaw = hardwareMap.get(Servo.class, "outtakeClaw1");

        // Retrieve the IMU from the hardware map
        IMU imu = hardwareMap.get(IMU.class, "imu");
        // Adjust the orientation parameters to match your robot
        IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.RIGHT,
                RevHubOrientationOnRobot.UsbFacingDirection.UP));
        // Without this, the REV Hub's orientation is assumed to be logo up / USB forward
        imu.initialize(parameters);


        intakeArm = hardwareMap.get(DcMotorEx.class, "intake1");
        intakeArm.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        intakeArm.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        intakeArm.setDirection(DcMotorEx.Direction.REVERSE);

        drivetrain.leftSlide.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        drivetrain.rightSlide.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);

        drivetrain.rightSlide.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        drivetrain.leftSlide.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);

        intakeArm.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        intakeArm.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);

        intakeController = new PIDController(p,i,d);
        moveIntake = 0;
        v4bTimer.reset();
        intakeClawTimer.reset();

        /** Distance Sensor Kalman Filter **/
        double initialEstimate = 4.65;  // Initial estimate based on the expected sensor reading
        double initialEstimateError = 1.0;  // Initial estimate error (start with a value that represents uncertainty)
        double processNoise = 0.1;  // Tune based on how much you expect the system to change between measurements
        double measurementNoise = 0.01;  // Tune based on the sensor's accuracy and environmental noise

        KalmanFilter kalmanFilter = new KalmanFilter(initialEstimate, initialEstimateError, processNoise, measurementNoise);


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
            if(gamepad2.back){
                drivetrain.leftSlide.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
                drivetrain.rightSlide.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);

                drivetrain.rightSlide.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
                drivetrain.leftSlide.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
            }




            /** MOVEMENT FOR FIELD CENTRIC AND AUTO ALIGN PIXEL **/
            double pixelDistance = sensorDistance.getDistance(DistanceUnit.INCH);
            kalmanFilter.update(pixelDistance);
            double filteredDistance = kalmanFilter.getEstimate();
//            double robotTheta = angleimu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.RADIANS).firstAngle;
            double robotTheta = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);
            double xError = getHorizontalDistance(cX, width);
            forwardPower = distanceController.calculate(0, filteredDistance);
            double horizontalpower = HorzPIDControl(HORZ_ERROR, getHorizontalDistance(cX, width));
            if(gamepad1.right_bumper) {
                drivetrain.power(AnglePIDControl(Math.toRadians(ANGLE_ERROR), robotTheta));
                if (filteredDistance > 12 && xError < 7) {
                    drivetrain.strafeToObject(horizontalpower);
                }
                if(gamepad1.y && filteredDistance > GRAB_DISTANCE){
                    drivetrain.moveRobot(forwardPower);
                }
            }else{
                drivetrain.moveRobotFieldCentric(y, x,rx,robotTheta);
            }

            /** INTAKE STATE MACHINE **/
            switch(intakeState) {
                case START:
                    if (gamepad1.b) {
                        intakeClaw.setPosition(CLOSE);
                        v4bTimer.reset();
                        intakeState = IntakeState.HOVER_PIXEL;
                    }
                    break;
                case RESET:
                    if (intakeClawTimer.seconds() >= INTAKE_CLAW_SERVO_TIME) {
                        intakeClaw.setPosition(CLOSE);
                        v4bTimer.reset();
                        intakeState = IntakeState.HOVER_PIXEL;
                    }
                    break;
                case HOVER_PIXEL:
                    if(v4bTimer.seconds() >= INTAKE_RESET_TIME) {
                        moveIntake = HOVER_PIXEL;
                        v4bTimer.reset();
                        intakeState = IntakeState.INT_PICKUP_PIXEL_ONE;
                    }
                    break;
                case INT_PICKUP_PIXEL_ONE:
                    if(v4bTimer.seconds() >= INTAKE_CLAW_RESET_TIME){
                        intakeClaw.setPosition(RELEASE_TWO);
                        v4bTimer.reset();
                        intakeState = IntakeState.INT_PICKUP_PIXEL;
                    }
                    break;
                case INT_PICKUP_PIXEL:
                    if(gamepad1.a){
                        moveIntake = INTAKE_PIXEL;
                        v4bTimer.reset();
                        intakeState = IntakeState.PICK_UP_PIXEL;
                    }
                    break;
                case PICK_UP_PIXEL:
                    if(v4bTimer.seconds() >= INTAKE_HOVER_GRAB_TIME){
                        intakeClaw.setPosition(CLOSE);
                        intakeClawTimer.reset();
                        intakeState = IntakeState.INT_TRANSFER_PIXEL;
                    }
                    break;
                case INT_TRANSFER_PIXEL:
                    if(intakeClawTimer.seconds() >= INTAKE_CLAW_SERVO_TIME){
                        v4bTimer.reset();
                        intakeState = IntakeState.TRANSFER_PIXEL;
                    }
                    break;
                case TRANSFER_PIXEL:
                    moveIntake = TRANSFER_PIXEL;
                    if(v4bTimer.seconds() >= INTAKE_TRANSFER_TIME){
                        intakeClaw.setPosition(RELEASE_TWO);
                        v4bTimer.reset();
                        intakeState = IntakeState.ARM_MOVE_UP;
                    }
                    break;
                case ARM_MOVE_UP:
                    if(v4bTimer.seconds() >= INTAKE_TRANSFER_TIME){
                        moveIntake = AVOID_PIXEL;
                        v4bTimer.reset();
                        intakeState = IntakeState.RESET;
                    }
                    break;
                default:
                    intakeState = IntakeState.START;
            }

            /** OUTTAKE STATE MACHINE **/

            switch(outtakeState) {
                case ARM_INIT:
                    leftServo.setPosition(LS_INIT);
                    rightServo.setPosition(RS_INIT);
                    outtakeClaw.setPosition(OUTTAKE_CLOSE);
                    outtakeArmTimer.reset();
                    outtakeState = OuttakeState.WRIST_INIT;
                    break;
                case WRIST_INIT:
                    if (outtakeArmTimer.seconds() >= WRIST_INIT_TIME) {
                        angledServo.setPosition(AS_INIT);
                        outtakeState = OuttakeState.HOVER_GRAB;
                    }
                    break;
                case HOVER_GRAB:
                    if (gamepad2.x) {
                        leftServo.setPosition(LS_GRAB);
                        rightServo.setPosition(RS_GRAB);

                        angledServo.setPosition(AS_GRAB);
                        outtakeArmTimer.reset();
                        outtakeState = OuttakeState.INIT_MOVE_GRAB_PIXEL;
//                        outtakeArmTimer.reset();
//                        outtakeState = OuttakeState.INIT_ANGLE_GRAB_PIXEL;
                    }
                    break;
                case INIT_ANGLE_GRAB_PIXEL:
                    if (outtakeArmTimer.seconds() >= HOVER_PIXEL_TIME) {
                        angledServo.setPosition(AS_GRAB);
                        outtakeArmTimer.reset();
                        outtakeState = OuttakeState.INIT_MOVE_GRAB_PIXEL;
                    }
                    break;
                case INIT_MOVE_GRAB_PIXEL:
                    if (outtakeArmTimer.seconds() >= GRAB_PIXEL_TIME) {
                        rightServo.setPosition(RS_GRAB2);
                        leftServo.setPosition(LS_GRAB2);
                        outtakeArmTimer.reset();
                        outtakeState = OuttakeState.OPEN;
                    }
                    break;
                case OPEN:
                    if (outtakeArmTimer.seconds() >= CLAW_MOVE_TIME) {
                        outtakeClaw.setPosition(RELEASE2);
                        outtakeArmTimer.reset();
                        outtakeState = OuttakeState.MOVE_DOWN;
                    }
                    break;
                case MOVE_DOWN:
                    if (outtakeArmTimer.seconds() >= CLAW_MOVE_TIME) {
                        leftServo.setPosition(LS_GRAB3);
                        rightServo.setPosition(RS_GRAB3);
                        outtakeArmTimer.reset();
                        outtakeState = OuttakeState.GRAB_PIXEL;
                    }
                    break;
                case GRAB_PIXEL:
                    if (outtakeArmTimer.seconds() >= CLAW_MOVE_TIME) {
                        outtakeClaw.setPosition(OUTTAKE_CLOSE);
                        outtakeArmTimer.reset();
                        outtakeState = OuttakeState.FLIP_ARM;
                    }
                    break;
                case FLIP_ARM:
                    if (outtakeArmTimer.seconds() >= OUTTAKE_CLAW_SERVO_TIME) {
                        leftServo.setPosition(LS_DEPOSIT);
                        rightServo.setPosition((RS_DEPOSIT));
                        outtakeArmTimer.reset();
                        outtakeState = OuttakeState.FLIP_WRIST;
                    }
                    break;
                case FLIP_WRIST:
                    if (outtakeArmTimer.seconds() >= FLIP_PIXEL_TIME) {
                        angledServo.setPosition(AS_DEPOSIT);
                        outtakeState = OuttakeState.DEPOSIT;
                    }
                    break;
                case DEPOSIT:
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
                        outtakeClaw.setPosition(RELEASE1);
                    }
                    if (gamepad2.left_bumper) {
                        outtakeClaw.setPosition(RELEASE2);
                    }
                    if (gamepad2.y) {
                        outtakeState = OuttakeState.ARM_INIT;
                    }
                    if (gamepad2.a) {
                        drivetrain.powerSlides(gamepad2.left_stick_y * slidesMultiplier);
                    } else if (drivetrain.getCurrentSlidePower() != 0) {
                        drivetrain.powerSlides(0);
                    }
                    break;
            }
            if(filteredDistance <= GRAB_DISTANCE){
                gamepad2.rumble(500);
            }

                intakeController.setPID(p, i, d);

            int armPos = intakeArm.getCurrentPosition();
            double intakePID = intakeController.calculate(armPos, moveIntake);

            double ff = Math.cos(Math.toRadians(moveIntake / ticks_in_degrees)) * f;

            double power = intakePID + ff;
            // Apply motion profiling to limit acceleration and jerk
            double maxDeltaV = maxAcceleration * runtime.seconds();
            double maxDeltaA = maxJerk * runtime.seconds();
//            power = clamp(power, -maxVelocity, maxVelocity, maxDeltaV, maxDeltaA);
            intakeArm.setPower(power);
            telemetry.addData("Intake State", intakeState);
            telemetry.addData("Intake Target Position", moveIntake);
            telemetry.addData("Intake Current Position", intakeArm.getCurrentPosition());
            telemetry.addData("Grab", filteredDistance < GRAB_DISTANCE);
            telemetry.addData("Raw Distance", pixelDistance);
            telemetry.addData("Filtered Distance", String.format("%.01f inch", filteredDistance));
            telemetry.addData("Vertical distance", getVerticalDistance(width));
            telemetry.addData("Horizontal distance", getHorizontalDistance(cX, width));
            telemetry.addData("Width", width);
            telemetry.addData("bot heading", Math.toDegrees(robotTheta));
            telemetry.addData("Left Slide Position", drivetrain.leftSlide.getCurrentPosition());
            telemetry.addData("Right Slide Position", drivetrain.rightSlide.getCurrentPosition());
            telemetry.update();
            runtime.reset();
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
                detectsObjects = true;
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
                detectsObjects = false;
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
