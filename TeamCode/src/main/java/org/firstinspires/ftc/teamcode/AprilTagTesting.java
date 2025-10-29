package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.stream.CameraStreamSource;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import java.util.List;

@TeleOp(name = "AprilTag Testing", group = "Robot")

public class AprilTagTesting extends OpMode {
    IMU imu;
    AprilTag aprilTag;

    @Override
    public void init() {
        imu = hardwareMap.get(IMU.class, "imu");
        // This needs to be changed to match the orientation on your robot
        RevHubOrientationOnRobot.LogoFacingDirection logoDirection =
                RevHubOrientationOnRobot.LogoFacingDirection.UP;
        RevHubOrientationOnRobot.UsbFacingDirection usbDirection =
                RevHubOrientationOnRobot.UsbFacingDirection.FORWARD;

        RevHubOrientationOnRobot orientationOnRobot = new
                RevHubOrientationOnRobot(logoDirection, usbDirection);
        imu.initialize(new IMU.Parameters(orientationOnRobot));

        aprilTag = new AprilTag("Webcam 1", hardwareMap);
    }

    @Override
    public void loop() {
        List<AprilTagDetection> currentDetections = aprilTag.getDetectedTags();
        telemetry.addData("AprilTags Detected", currentDetections.size());
        for (AprilTagDetection detection : currentDetections) {
            if (detection.metadata != null) {
                switch (detection.id) {
                    case 20:
                        telemetry.addLine("Found blue GOAL");
                        break;
                    case 24:
                        telemetry.addLine("Found red GOAL");
                        break;
                    case 21:
                        telemetry.addData("Pattern", "GPP");
                        break;
                    case 22:
                        telemetry.addData("Pattern", "PGP");
                        break;
                    case 23:
                        telemetry.addData("Pattern", "PPG");
                        break;
                }
            }
        }
    }
}
