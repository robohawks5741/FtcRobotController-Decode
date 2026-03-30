package org.firstinspires.ftc.teamcode.opmodes.test;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;

@TeleOp(name="motorTest")
public class motorTester extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        DcMotorEx motor = hardwareMap.get(DcMotorEx.class, "motor");
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
