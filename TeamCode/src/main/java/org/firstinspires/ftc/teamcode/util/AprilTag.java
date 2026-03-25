package org.firstinspires.ftc.teamcode.util;

import android.util.Size;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.hardware.camera.BuiltinCameraDirection;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.List;

public class AprilTag {
    private static final boolean USE_WEBCAM = true;

    private AprilTagProcessor aprilTag;
    private VisionPortal visionPortal;

    public AprilTag(String deviceName, HardwareMap hardwareMap) {
        if (deviceName.isEmpty()) throw new RuntimeException("Must provide a device name (AprilTag)");
        aprilTag = new AprilTagProcessor.Builder()
                .setDrawAxes(true)
                .setDrawCubeProjection(true)
                .setDrawTagOutline(true)
                .setOutputUnits(DistanceUnit.INCH, AngleUnit.DEGREES)
               // .setCameraPose()
                .build();

        VisionPortal.Builder builder = new VisionPortal.Builder();
        builder.setCamera(hardwareMap.get(WebcamName.class, deviceName));
        builder.setLiveViewContainerId(0);
        builder.setStreamFormat(VisionPortal.StreamFormat.YUY2);
        builder.setAutoStopLiveView(true);
        builder.addProcessor(aprilTag);
        builder.setCameraResolution(new Size(1920, 1080));

        visionPortal = builder.build();
        visionPortal.setProcessorEnabled(aprilTag, true);
    }

    public List<AprilTagDetection> getDetectedTags() {
        return aprilTag.getDetections();
    }

    public void setDecimation(int decimation) {
        if (decimation < 1 || decimation > 3)
            throw new RuntimeException("Decimation must be between 1 and 3");
        aprilTag.setDecimation(decimation);
    }
}
