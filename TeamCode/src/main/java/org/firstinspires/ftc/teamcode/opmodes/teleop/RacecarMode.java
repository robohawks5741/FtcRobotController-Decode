package org.firstinspires.ftc.teamcode.opmodes.teleop;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.DcMotorEx;

@TeleOp(name="Racecar Mode")
public class RacecarMode extends LinearOpMode {
    protected DcMotorEx backLeftMotor;
    protected DcMotorEx backRightMotor;

    protected Servo steeringServo;

    @Override
    public void runOpMode() {
        backLeftMotor = hardwareMap.get(DcMotorEx.class, "backLeft");
        backRightMotor = hardwareMap.get(DcMotorEx.class, "backRight");
        steeringServo = hardwareMap.get(Servo.class, "steeringServo");

        backLeftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backRightMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        telemetry.addLine("Racecar Mode, not for use on primary robot");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {
            backLeftMotor.setPower(gamepad1.left_stick_y);
            backRightMotor.setPower(gamepad1.left_stick_y);
            steeringServo.setPosition(gamepad1.right_stick_x);
        }
    }
}