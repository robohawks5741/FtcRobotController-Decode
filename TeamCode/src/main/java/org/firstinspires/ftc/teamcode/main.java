package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.MecanumDrive;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;

import java.util.List;

@TeleOp(name="MAIN")
public class main extends  robot{
    public void loop() {
        PinpointLocalizer.update();
        List<AprilTagDetection> currentDetections = aprilTag.getDetectedTags();
        telemetry.addData("AprilTags Detected", currentDetections.size());
        // If you press the A button, then you reset the Yaw to be zero from the way
        // the robot is currently pointing

        // If you press the left bumper, you get a drive from the point of view of the robot
        // (much like driving an RC vehicle)
        if (gamepad1.left_bumper) {
            driveFieldRelative(gamepad1.left_stick_y, gamepad1.left_stick_x, gamepad1.right_stick_x);
        } else {
            drive.setDrivePowers(new PoseVelocity2d(
                    new Vector2d(
                            -gamepad1.left_stick_y,
                            -gamepad1.left_stick_x
                    ),
                    -gamepad1.right_stick_x
            ));
        }
        if (gamepad1.x) {
            driveto(10, 10, 10);
        }
        Double launchPow;
        if (gamepad1.right_trigger >= 0.1) {
            launchPow = 1.0;
        } else {
            launchPow = 0.0;
        }
        launcher1.setPower(-launchPow);
        launcher2.setPower(launchPow);
        if (gamepad1.a) {
            launchFeed.setPower(1);
        } else {
            launchFeed.setPower(0);
        }
        if (gamepad1.b) {
            if (aprilTag != null) {
                for (AprilTagDetection detection : aprilTag.getDetectedTags()) {
                    if (detection.id == 20 || detection.id == 24) {
                        driveto(globalLoc().position.x, globalLoc().position.y, detection.ftcPose.bearing);
                    }
                }
            }

        }
        double turnFactor = gamepad1.right_stick_x;
        if (gamepad1.y) {
            if (!aprilTag.getDetectedTags().isEmpty()) {
                for (AprilTagDetection detection : aprilTag.getDetectedTags()) {
                    if (detection.id == 20 || detection.id == 24) {
                        telemetry.addLine("GOAL visible");
                        if (detection.center.x < 400) turnFactor += 0.5;
                        if (detection.center.x > 400) turnFactor -= 0.5;

                    }
                }
            }
            drive.setDrivePowers(new PoseVelocity2d(
                    new Vector2d(
                            -gamepad1.left_stick_y,
                            -gamepad1.left_stick_x
                    ),
                    turnFactor
            ));
        }
        telemetry.addData("globalLocx", globalLoc().position.x);
        telemetry.addData("globalLocy", globalLoc().position.y);
        telemetry.addData("globalLoctheta", Math.toDegrees(globalLoc().heading.toDouble()));
        telemetry.addData("podx", PinpointLocalizer.driver.getPosX(DistanceUnit.MM));
        telemetry.update();
        updateTelemetry(telemetry);
    }
}
