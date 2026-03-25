package org.firstinspires.ftc.teamcode.opmodes.test;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.robot;

@TeleOp(name = "#RESET TELEOP BEGIN POSE", group = "robot")
public class resetBeginPose extends robot{
    @Override
    public void runOpMode() throws InterruptedException {
        waitForStart();
        teleOpBeginPose = null;
        hood = hardwareMap.get(Servo.class, "hood");
        while (opModeIsActive()){
            telemetry.addData("teleOpBeginPose should = null", "");
            telemetry.addData("teleOpBeginPose", teleOpBeginPose);
            hoodPosition = 0.95;
            hood.setPosition(hoodPosition);
            if (isStopRequested()) {
                break;
            }
        }
    }
}
