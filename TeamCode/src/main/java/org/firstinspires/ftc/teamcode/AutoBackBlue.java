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

    }
    public static Params PARAMS = new Params();

    //CRServo turret1;
    //CRServo turret2;
    PID pid;
    Limelight3A limelight;
    boolean redOverride = false;
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
        boolean waiting = true;
        super.runOpMode();
        if (!redOverride) {
            isRedAlliance = false;
        }
        telemetry.addData("Status:", " Initialized");
        //super.runOpMode();
        //Pose2d beginPose = new Pose2d(robot.PARAMS.beginPosX, robot.PARAMS.beginPosY, Math.toRadians(180));
        teleOpBeginPose = beginPos;
       // AprilTag aprilTag = new AprilTag("Webcam 1", hardwareMap);
        boolean beginPoseFound = true;
     //   aprilTag.getDetectedTags().;
       // limelight.pipelineSwitch(1);
        while (!beginPoseFound && opModeInInit()) {
            if (result.isValid()) {
                //beginPos = new Pose2d(new Vector2d(result.getBotpose().getPosition().x, result.getBotpose().getPosition().y), result.getBotpose().getOrientation().getYaw());

                telemetry.addData("x", beginPos.position.x);
                telemetry.addData("y", beginPos.position.y);
                telemetry.addData("heading", Math.toDegrees(beginPos.heading.toDouble()));
                telemetry.update();
                beginPoseFound = true;
            }

        }
       // limelight.pipelineSwitch(0);
        telemetry.update();

        //MecanumDrive drive = new MecanumDrive(hardwareMap, beginPos);
        //drive.defaultAccelConstraint.
        //PinpointLocalizer pinpoint = new PinpointLocalizer(hardwareMap, 0.0007669904, beginPose);
        
        waitForStart();
        if (opModeIsActive()) {
            if (isStopRequested()) return;
            telemetry.addData("Status: ", "Running");
            autoPower = 2150;
            setLaunchRPM(autoPower);
            hood.setPosition(0.25);
            drive.localizer.setPose(beginPos);
          /*  Actions.runBlocking(
                    drive.actionBuilder(drive.localizer.getPose())
                            .lineToYConstantHeading(10)
                            // .strafeTo(new Vector2d(0, 0))
                            .turn(Math.toRadians(90))
                            .build()
            );*/
            drive.localizer.update();
            //drive.localizer.getPose();
           // hood.setPosition(0.0);
            double time = 0;
            //power = 4000;
            Actions.runBlocking(new SequentialAction(
                    new rowSelectAuto(4),
                    new newLaunchCycle(true, false),
                    new rowSelectAuto(2),
                    new newLaunchCycle(true, false),
                    new rowSelectAuto(3),
                    new newLaunchCycle(true, false),
                    new sendAutoEndPose()
                   // new rowSelectAuto(4)
                   /* new rowSelectAuto(3),
                    new newLaunchCycle(true)*/

                ));
            }else {
                Actions.runBlocking(new SequentialAction(
                        new rowSelectAuto(4),
                        new newLaunchCycle(true, false),
                        new rowSelectAuto(2),
                        new newLaunchCycle(true, false),
                        new rowSelectAuto(3),
                        new newLaunchCycle(true, false),
                        new sendAutoEndPose()
                        // new rowSelectAuto(4)
                   /* new rowSelectAuto(3),
                    new newLaunchCycle(true)*/

                ));
            }
            if (isStopRequested()) {
                new sendAutoEndPose();
            }
            telemetry.addData("TeleOpBeginPose", teleOpBeginPose);
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

