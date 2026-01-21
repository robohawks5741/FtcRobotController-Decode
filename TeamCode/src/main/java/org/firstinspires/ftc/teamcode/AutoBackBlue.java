package org.firstinspires.ftc.teamcode;

import static java.lang.Math.abs;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.subsystems.MecanumDrive;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;

import java.security.cert.CRL;

@Autonomous(name = "AutoBackBlue", group = "Robot")
public class AutoBackBlue extends LinearOpMode {
    public static class Params {
        public double beginPosX = 62;
        public double beginPosY = -22.5;
        public double Kp = 0.0045;
        public double Ki = 0.0001028;
        //double Kd = 0.000000045;
        public double Kd = 0.0000135;
    }
    public static Params PARAMS = new Params();

    CRServo turret1;
    CRServo turret2;
    PID pid;
    Limelight3A limelight;
    public class turretTrack implements Action {
        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            while (opModeIsActive()) {
                turret2.setPower(-pid.PIDControl(PARAMS.Kp, PARAMS.Ki, PARAMS.Kd, 0.0, limelight.getLatestResult().getTx()));
                if (abs(limelight.getLatestResult().getTx()) < 2) {
                    break;
                }
            }
            return false;
        }
    }
    @Override
    public void runOpMode() {
        telemetry.addData("Status:", " Initialized");
        //super.runOpMode();
        Pose2d beginPose = new Pose2d(PARAMS.beginPosX, PARAMS.beginPosY, Math.toRadians(180));
        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        LLResult result = limelight.getLatestResult();
        pid = new PID();
       // AprilTag aprilTag = new AprilTag("Webcam 1", hardwareMap);
        boolean beginPoseFound = false;
     //   aprilTag.getDetectedTags().;
        limelight.pipelineSwitch(1);
        while (!beginPoseFound && opModeInInit()) {
            if (result.isValid()) {
                beginPose = new Pose2d(new Vector2d(result.getBotpose().getPosition().x - 0.01, result.getBotpose().getPosition().y), result.getBotpose().getOrientation().getYaw());
                beginPoseFound = true;
            }
        }
        limelight.pipelineSwitch(0);
        telemetry.update();

        MecanumDrive drive = new MecanumDrive(hardwareMap, beginPose);
        //PinpointLocalizer pinpoint = new PinpointLocalizer(hardwareMap, 0.0007669904, beginPose);
        
        waitForStart();
        if (opModeIsActive()) {
            if (isStopRequested()) return;
            telemetry.addData("Status: ", "Running");
            drive.localizer.setPose(beginPose);
          /*  Actions.runBlocking(
                    drive.actionBuilder(drive.localizer.getPose())
                            .lineToYConstantHeading(10)
                            // .strafeTo(new Vector2d(0, 0))
                            .turn(Math.toRadians(90))
                            .build()
            );*/
            drive.localizer.update();
            beginPose = drive.localizer.getPose();

            double time = 0;
            Actions.runBlocking(new ParallelAction(
                    drive.actionBuilder(beginPose)
                            //.splineToConstantHeading(new Vector2d(-50, 0), 0)
                            .strafeTo(new Vector2d(0,-22.5))
                            .endTrajectory()
                            .strafeTo(new Vector2d(62,-22.5))

                            //.endTrajectory()
                            //.splineToConstantHeading(new Vector2d(0,0), Math.toRadians(0))
                            .build(), new turretTrack()));
            telemetry.addData("Status: ", "Done");
            telemetry.addData("PPX", drive.localizer.getPose().position.x);
            telemetry.addData("PPY", drive.localizer.getPose().position.y);
            telemetry.addData("PPRot", Math.toDegrees(drive.localizer.getPose().heading.toDouble()));
            telemetry.update();
            while (opModeIsActive()) {
                if (time > 20) {
                    telemetry.addData("Status: ", "Idle - Telemetry");
                    telemetry.addData("PPX", drive.localizer.getPose().position.x);
                    telemetry.addData("PPY", drive.localizer.getPose().position.y);
                    telemetry.addData("PPRot", drive.localizer.getPose().heading.toDouble());
                    telemetry.update();
                }
            }
        }
    }
}
