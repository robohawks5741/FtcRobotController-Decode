package org.firstinspires.ftc.teamcode;

import static androidx.core.math.MathUtils.clamp;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;

@TeleOp(name="launchTest")
public class launchTester extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        DcMotorEx motor = hardwareMap.get(DcMotorEx.class, "motor");
        Servo hood = hardwareMap.get(Servo.class, "hood");
        CRServo feed = hardwareMap.get(CRServo.class, "feed");
        motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motor.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        motor.setVelocityPIDFCoefficients(0.3,0.0,0.3, 4);

        waitForStart();
        while (opModeIsActive()) {
            if (gamepad1.right_trigger > 0.0) {
                motor.setVelocity(-gamepad1.right_stick_y * 6000, AngleUnit.DEGREES);
            } else if (gamepad1.left_trigger > 0.0) {
                motor.setPower(gamepad1.right_stick_y);
            } else if (gamepad1.right_bumper){
                motor.setVelocity(6000, AngleUnit.DEGREES);
            }else {
                motor.setPower(0.0);
            }
            if (gamepad1.left_bumper){
                hood.setPosition(clamp(gamepad1.left_stick_y, -0.5, -0.1));
            }
            if (gamepad1.y) {
                feed.setPower(1);
            }
            telemetry.addData("Power", motor.getPower());
            telemetry.addData("RPM", motor.getVelocity(AngleUnit.DEGREES));
            telemetry.addData("RPM Adjusted", motor.getVelocity(AngleUnit.DEGREES)*20);
            telemetry.addData("TargetRPM", gamepad1.right_stick_y*6000);
            telemetry.addData("Right Stick Y", gamepad1.right_stick_y);
            //telemetry.addData("Left Trigger", gamepad1.left_trigger);
            telemetry.addData("Current", motor.getCurrent(CurrentUnit.AMPS));
            telemetry.update();

        }
    }


}
