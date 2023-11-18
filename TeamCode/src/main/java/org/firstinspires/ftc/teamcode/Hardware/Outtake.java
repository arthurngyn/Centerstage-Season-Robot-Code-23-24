package org.firstinspires.ftc.teamcode.Hardware;

import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;


public class Outtake {
    private DcMotorEx leftSlide;
    private DcMotorEx rightSlide;
    private DcMotorEx intakeArm;
    private Servo leftServo;
    private Servo rightServo;
    private Servo angledServo;
    private Servo clawServo;

    HardwareMap hwMap;

    public void init(HardwareMap ahwMap) {

        /**
         * Assigns the parent hardware map to local ArtemisHardwareMap class variable
         * **/
        hwMap = ahwMap;

        /**
         * Hardware initialized and String Names are in the Configuration File for Hardware Map
         * **/

        // Control HUb
        leftSlide = hwMap.get(DcMotorEx.class, "Left-Slide2");
        rightSlide = hwMap.get(DcMotorEx.class, "Right-Slide3");
        intakeArm = hwMap.get(DcMotorEx.class, "intake0");
        leftServo = hwMap.get(Servo.class, "Left2");
        rightServo = hwMap.get(Servo.class, "Right3");
        angledServo = hwMap.get(Servo.class, "Angle1");
        clawServo = hwMap.get(Servo.class, "Claw0");

        /**
         * Allow the 2 slide motors to be run without encoders since we are doing a time based autonomous
         * **/
        leftSlide.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        rightSlide.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        intakeArm.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        /**
         * We are setting the motor 0 mode power to be brake as it actively stops the robot and doesn't rely on the surface to slow down once the robot power is set to 0
         * **/
        rightSlide.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        leftSlide.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        intakeArm.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        intakeArm.setDirection(DcMotorEx.Direction.REVERSE);


    }
}
