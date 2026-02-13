package org.firstinspires.ftc.teamcode;

import static java.lang.Math.abs;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.subsystems.MecanumDrive;

@Autonomous(name = "AutoBackBlue", group = "Robot")
public class AutoBackBlue extends robot {
    public static class Params {
        public double beginPosX = 62;
        public double beginPosY = -22.5;
        public double targetX = -24;
        public double targetY = -34;
        public double endX = 30;
        public double endY = -20;
        public double targetHeading = -135;
    }
    public static Params PARAMS = new Params();

    //CRServo turret1;
    //CRServo turret2;
    PID pid;
    Limelight3A limelight;
  /*  public class turretTrack implements Action {
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
    }*/
    @Override
    public void runOpMode() throws InterruptedException {
        super.runOpMode();
        telemetry.addData("Status:", " Initialized");
        //super.runOpMode();
        Pose2d beginPose = new Pose2d(PARAMS.beginPosX, PARAMS.beginPosY, Math.toRadians(180));

       // AprilTag aprilTag = new AprilTag("Webcam 1", hardwareMap);
        boolean beginPoseFound = false;
     //   aprilTag.getDetectedTags().;
       // limelight.pipelineSwitch(1);
        while (!beginPoseFound && opModeInInit()) {
            if (result.isValid()) {
                beginPose = new Pose2d(new Vector2d(result.getBotpose().getPosition().x - 0.01, result.getBotpose().getPosition().y), result.getBotpose().getOrientation().getYaw());

                telemetry.addData("x", beginPose.position.x);
                telemetry.addData("y", beginPose.position.y);
                telemetry.addData("heading", Math.toDegrees(beginPose.heading.toDouble()));
                telemetry.update();
                beginPoseFound = true;
            }

        }
       // limelight.pipelineSwitch(0);
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
           // hood.setPosition(0.0);
            double time = 0;
            Actions.runBlocking(new SequentialAction(
                    drive.actionBuilder(beginPose)
                            //.splineToConstantHeading(new Vector2d(-50, 0), 0)
                            .strafeTo(new Vector2d(PARAMS.targetX,PARAMS.targetY))
                            .endTrajectory()
                            .turnTo(Math.toRadians(PARAMS.targetHeading))
                            .endTrajectory()
                            //.strafeTo(new Vector2d(62,-22.5))

                            //.endTrajectory()
                            //.splineToConstantHeading(new Vector2d(0,0), Math.toRadians(0))
                            .build(),  new newLaunchCycle(), drive.actionBuilder(drive.localizer.getPose()).turnTo(Math.toRadians(0)).strafeTo(new Vector2d(PARAMS.endX,PARAMS.endY)).endTrajectory().build())
            );
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
