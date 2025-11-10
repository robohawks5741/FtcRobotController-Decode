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
public class main extends robot {
    public void loop() {
        double turnValue = -gamepad1.right_stick_x;

        // If you press the A button, then you reset the Yaw to be zero from the way
        // the robot is currently pointing

        if (gamepad1.a) {
            imu.resetYaw();
        }

        PinpointLocalizer.update();
        // If you press the A button, then you reset the Yaw to be zero from the way
        // the robot is currently pointing

        // If you press the left bumper, you get a drive from the point of view of the robot
        // (much like driving an RC vehicle)

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

        List<AprilTagDetection> currentDetections = aprilTag.getDetectedTags();

        if (gamepad1.b) { // Juniper AprilTag code
            if (aprilTag != null) {
                for (AprilTagDetection detection : currentDetections) {
                    if (detection.id == 20 || detection.id == 24) {
                        driveto(globalLoc().position.x, globalLoc().position.y, detection.ftcPose.bearing);
                    }
                }
            }

        }

        if (gamepad1.dpad_left) { // Aidan AprilTag code
            for (AprilTagDetection detection : currentDetections) {
                if (detection.id == 20 || detection.id == 24) {
                    telemetry.addLine("GOAL visible");
                    double turnVector = pid.PIDControl(0.5, 0.1, 0.1, 400, detection.center.x);
                    turnValue += turnVector;
                    telemetry.addData("Turn vector from PID: ", turnVector);
                }
            }
        }

        if (gamepad1.left_bumper) {
            driveFieldRelative(gamepad1.left_stick_y, gamepad1.left_stick_x, gamepad1.right_stick_x);
        } else {
            double turnFactor = gamepad1.right_stick_x;
            if (gamepad1.y) { // Aidan primitive AprilTag code
                if (!currentDetections.isEmpty()) {
                    for (AprilTagDetection detection : currentDetections) {
                        if (detection.id == 20 || detection.id == 24) {
                            telemetry.addLine("GOAL visible");
                            if (detection.center.x < 400) turnFactor += 0.5;
                            if (detection.center.x > 400) turnFactor -= 0.5;

                        }
                    }
                }
            }
            drive.setDrivePowers(new PoseVelocity2d(
                    new Vector2d(
                            -gamepad1.left_stick_y,
                            -gamepad1.left_stick_x
                    ),
                    turnValue
            ));
        }
        telemetry.addData("globalLoc X", globalLoc().position.x);
        telemetry.addData("globalLoc Y", globalLoc().position.y);
        telemetry.addData("globalLoc theta", Math.toDegrees(globalLoc().heading.toDouble()));
        telemetry.addData("pinpoint X", PinpointLocalizer.driver.getPosX(DistanceUnit.MM));
        telemetry.update();
        updateTelemetry(telemetry);
    }
}
