package org.firstinspires.ftc.teamcode.TeleOp;

import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name = "Intake Test")
public class intakeTest extends LinearOpMode {

    private DcMotorEx intakeMotor;
    @Override
    public void runOpMode() throws InterruptedException {
        intakeMotor = hardwareMap.get(DcMotorEx.class, "intakeMotor0");
        waitForStart();
        while(opModeIsActive()) {
                intakeMotor.setPower(gamepad1.right_trigger);
        }
    }
}
