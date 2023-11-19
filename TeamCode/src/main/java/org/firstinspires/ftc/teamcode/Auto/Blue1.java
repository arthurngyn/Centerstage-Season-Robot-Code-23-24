package org.firstinspires.ftc.teamcode.Auto;

import static org.firstinspires.ftc.teamcode.Hardware.Variables.IntakeArmVariables.INTAKE_PIXEL;
import static org.firstinspires.ftc.teamcode.Hardware.Variables.IntakeArmVariables.START;
import static org.firstinspires.ftc.teamcode.Hardware.Variables.IntakeArmVariables.d;
import static org.firstinspires.ftc.teamcode.Hardware.Variables.IntakeArmVariables.da;
import static org.firstinspires.ftc.teamcode.Hardware.Variables.IntakeArmVariables.f;
import static org.firstinspires.ftc.teamcode.Hardware.Variables.IntakeArmVariables.i;
import static org.firstinspires.ftc.teamcode.Hardware.Variables.IntakeArmVariables.p;
import static org.firstinspires.ftc.teamcode.Hardware.Variables.IntakeArmVariables.pa;
import static org.firstinspires.ftc.teamcode.Hardware.Variables.IntakeClawVariables.CLOSE;
import static org.firstinspires.ftc.teamcode.Hardware.Variables.IntakeClawVariables.RELEASE_TWO;
import static org.firstinspires.ftc.teamcode.Hardware.Variables.IntakeTimeVariables.INTAKE_CLAW_RESET_TIME;
import static org.firstinspires.ftc.teamcode.Hardware.Variables.IntakeTimeVariables.INTAKE_CLAW_SERVO_TIME;

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
import com.qualcomm.robotcore.robot.Robot;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.TeleOp.DriveToPixel;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.teamcode.util.ErrorCalc;

@Config
@Autonomous(name = "Blue 1", group = "Blue")
public class Blue1 extends LinearOpMode {

    public static double SPIKE_MARK_X = -22.5;
    public static double SPIKE_MARK_Y = 33;
    public static double SPIKE_MARK_ANGLE = 0;
    public static double BACKDROP_X = -32;
    public static double BACKDROP_Y = 20;
    public static double BACKDROP_ANGLE = 0;

    public static double xyP = 0.5;
    public static double headingP = 0.5;

    public enum RobotState {
        START,
        DRIVE_TO_SPIKE,
        DROP_SPIKE,
        RESET_INTAKE,
        DRIVE_TO_BACKDROP,
        LOCK_POSITION
    }
    RobotState robot = RobotState.START;
    private Servo intakeClaw;
    private DcMotorEx intakeArm;
    private ElapsedTime intakeClawTimer = new ElapsedTime();
    private ElapsedTime v4bTimer = new ElapsedTime();
    private PIDController intakeController;
    public static int moveIntake = 0;




    @Override
    public void runOpMode() throws InterruptedException {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);


        ErrorCalc calculate = new ErrorCalc();

        /** INTAKE ARM PIDF **/
        intakeClaw = hardwareMap.get(Servo.class, "intakeClaw0");
        intakeArm = hardwareMap.get(DcMotorEx.class, "intake1");
        intakeArm.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        intakeArm.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        intakeArm.setDirection(DcMotorEx.Direction.REVERSE);
        intakeController = new PIDController(pa, i, da);
        moveIntake = 0;
        v4bTimer.reset();
        intakeClawTimer.reset();


        Pose2d startPose = new Pose2d(0, 0, Math.toRadians(90));
        drive.setPoseEstimate(startPose);
        Trajectory DROP_SPIKE = drive.trajectoryBuilder(startPose)
                .lineToSplineHeading(new Pose2d(SPIKE_MARK_X, SPIKE_MARK_Y, Math.toRadians(SPIKE_MARK_ANGLE)))
                .build();
        Trajectory DROP_PIXEL = drive.trajectoryBuilder(DROP_SPIKE.end())
                .lineToSplineHeading(new Pose2d(BACKDROP_X, BACKDROP_Y, Math.toRadians(BACKDROP_ANGLE)))
                .build();


        waitForStart();

        if (isStopRequested()) return;
        Pose2d poseEstimate = drive.getPoseEstimate();

        while (opModeIsActive() && !isStopRequested()) {
            switch (robot) {
                case START:
                    intakeClaw.setPosition(CLOSE);
                    robot = RobotState.DRIVE_TO_SPIKE;
                    break;
                case DRIVE_TO_SPIKE:
                    drive.followTrajectory(DROP_SPIKE);
                    if (!drive.isBusy()) {
                        v4bTimer.reset();
                        robot = RobotState.DROP_SPIKE;
                    }
                    break;
                case DROP_SPIKE:
                    moveIntake = INTAKE_PIXEL;
                    if (v4bTimer.seconds() >= INTAKE_CLAW_SERVO_TIME) {
                        intakeClaw.setPosition(RELEASE_TWO);
                        intakeClawTimer.reset();
                        robot = RobotState.RESET_INTAKE;
                    }
                    break;
                case RESET_INTAKE:
                    if (intakeClawTimer.seconds() >= INTAKE_CLAW_RESET_TIME) {
                        moveIntake = START;
                        intakeClaw.setPosition(CLOSE);
                        robot = RobotState.DRIVE_TO_BACKDROP;
                    }
                    break;
                case DRIVE_TO_BACKDROP:
                    drive.followTrajectory(DROP_PIXEL);
                    if(!drive.isBusy()){
                        robot = RobotState.LOCK_POSITION;
                    }
                    break;
                case LOCK_POSITION:
                    lockTo(new Pose2d(BACKDROP_X, BACKDROP_Y,Math.toRadians(BACKDROP_ANGLE)), drive);
                    drive.update();
            }


            intakeController.setPID(pa, i, da);
            int armPos = intakeArm.getCurrentPosition();
            double intakePID = intakeController.calculate(armPos, moveIntake);
            double power = intakePID;
            intakeArm.setPower(power);

            telemetry.addData("Robot State: ", robot);
            telemetry.addData("Intake Position", intakeArm.getCurrentPosition());
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

}
