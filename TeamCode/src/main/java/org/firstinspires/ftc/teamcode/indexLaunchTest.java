package org.firstinspires.ftc.teamcode;

import static androidx.core.math.MathUtils.clamp;
import static java.lang.Math.abs;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;

import java.util.concurrent.TimeUnit;

@TeleOp(name="##Tele-Red")
public class indexLaunchTest extends robot {

    // A timer to calculate the change in time (delta time)
    ElapsedTime timer = new ElapsedTime();
    ElapsedTime changedTimer = new ElapsedTime();
    long lastChangedTime = 0;
    boolean isTeleOpRed = true;

    @Override
    public void runOpMode() throws InterruptedException {
        //DcMotorEx motor = hardwareMap.get(DcMotorEx.class, "motor");

        isRedAlliance = isTeleOpRed;
        super.runOpMode();
        if (teleOpBeginPose == null) {
            teleOpBeginPose = beginPos;
        }
      //  launcher.setVelocityPIDFCoefficients(0.4,0.001,0.3, 4);
        double Kp = PARAMS.Kp;
        double Ki = PARAMS.Ki;
        //double Kd = 0.000000045;
        double Kd = PARAMS.Kd;
        double changingValue = 0;

        boolean x = false;
        boolean launchTriggered = false;
        boolean dpadLeftPressed = false;
        boolean dpadRightPressed = false;
        boolean indexUp = false;
        boolean indexDown = false;

        Pose2d beginPose;
        //yaw of turret in robot space, 0 is center, positive is right, negative is left
        double turretYawRobot = 0.0;
        double turretYawRobot2 = 0.0;
        PID pid = new PID();
        double TX = 0;
       // AprilTag aprilTag = new AprilTag("Webcam 1", hardwareMap);
        boolean beginPoseFound = false;
        Limelight3A limelight = hardwareMap.get(Limelight3A.class, "limelight");

        telemetry.setMsTransmissionInterval(10);

        limelight.pipelineSwitch(0);
        double gValue;
        double rValue;
        double bValue;

        /*
         * Starts polling for data.  If you neglect to call start(), getLatestResult() will return null.
         */
        limelight.start();
      //  LLResult result = limelight.getLatestResult();
        while (opModeInInit()) {

        }
        //   aprilTag.getDetectedTags().;
        /*while (!beginPoseFound) {
            for (AprilTagDetection tag : aprilTag.getDetectedTags()) {
                //Todo: change detect ID to blue (20)
                if (tag.id == 24) {
                    beginPose = new Pose2d(tag.robotPose.getPosition().x, tag.robotPose.getPosition().y, tag.robotPose.getOrientation().getYaw(AngleUnit.RADIANS));
                    beginPoseFound = true;
                    telemetry.addData("Pose (from camera/april tag):", "");
                    telemetry.addData("X", beginPose.position.x);
                    telemetry.addData("Y", beginPose.position.y);
                }
            }
        }*/
        telemetry.update();
      //  MecanumDrive drive = new MecanumDrive(hardwareMap, new Pose2d(new Vector2d(0,0), 0));
     //b   limelight.pipelineSwitch(0);
        waitForStart();
        indexer(1);
        indexer(0);

        drive.localizer.setPose(teleOpBeginPose);
        setTurretPosition(targetTurretAngle);
        while (opModeIsActive()) {
            drive.updatePoseEstimate();
            result = limelight.getLatestResult();

            drive.localizer.update();
            robot.PARAMS.localizerX = drive.localizer.getPose().position.x;
            robot.PARAMS.localizerY = drive.localizer.getPose().position.y;
            blueGoalHeading = Math.toDegrees(Math.atan2(blueGoalY -PARAMS.localizerY, blueGoalX -PARAMS.localizerX));
            blueGoalDistance = Math.hypot(blueGoalX -PARAMS.localizerX, blueGoalY -PARAMS.localizerY);
            redGoalHeading = Math.toDegrees(Math.atan2(redGoalY -PARAMS.localizerY, redGoalX -PARAMS.localizerX));
            redGoalDistance = Math.hypot(redGoalX -PARAMS.localizerX, redGoalY -PARAMS.localizerY);
           // drive.localizer.setPose(result.getBotpose());
            YawPitchRollAngles orientation = imu.getRobotYawPitchRollAngles();
            limelight.updateRobotOrientation(orientation.getYaw());
            colorLogger();
            indexLoadManagement();

            if (gamepad1.dpad_left || gamepad2.dpad_left && !dpadLeftPressed) {
                totalTurretRotations -=1;
                dpadLeftPressed = true;
            }else if (gamepad1.dpad_right || gamepad2.dpad_right && !dpadRightPressed){
                totalTurretRotations +=1;
                dpadRightPressed = true;
            }
            if (!gamepad1.dpad_left && !gamepad2.dpad_left){
                dpadLeftPressed = false;
            }
            if (!gamepad1.dpad_right && !gamepad2.dpad_right){
                dpadRightPressed = false;
            }

            if (gamepad2.circle && !indexUp) {
                index += 2;
                if (index > 4) index = 0;
                indexUp = true;
            }else if (gamepad2.square && !indexDown){
                index -= 2;
                if (index < 0) index = 4;
                indexDown = true;
            }
            if (!gamepad2.circle){
                indexUp = false;
            }
            if (gamepad2.square){
                indexDown = false;
            }

            if (gamepad1.right_bumper || gamepad2.right_bumper) {
                targetTurretAngle -= gamepad1.right_stick_x*8;
                turretPID.turretReset();
                //PARAMS.turretAngle = 180;
            } else if (gamepad1.left_stick_button){
                targetTurretAngle = 45;
            } else {

                new turretTrack(isRedAlliance).run(new TelemetryPacket());
                if (turret1.getPower() <0.1) {
                    turretPID.turretReset();
                }

                // turret2.setPower(0);
            }
            if (gamepad1.share || gamepad2.share) {
                targetTurretAngle = 0;
            }
            if (result.isValid() && result != null) {
                telemetry.addData("Target Locked?", true);
            }
            telemetry.addData("Updating Pose? ", updatePoseFromLimeLight());
            drive.localizer.update();

            if (gamepad1.dpad_up){
                liftL.setPower(1);
                liftR.setPower(1);
            } else if (gamepad1.dpad_down) {
                liftL.setPower(-1);
                liftR.setPower(-1);
            } else {
                liftL.setPower(0);
                liftR.setPower(0);
            }
            if (gamepad1.right_trigger > 0.0) {

                setLaunchRPM(teleopPower);
               //launcher.setVelocity(150, AngleUnit.DEGREES);
            }else {
                setLaunchRPM(0);
            }
            if (gamepad1.left_trigger > 0 /*&& index%2 == 1*/) {
                feedOn();
                /*launchFeedL.setPower(-1);
                launchFeedR.setPower(1);
                feed.setPosition(0.05);*/
            } else {
                feedOff();
            }
            if (gamepad1.left_bumper){
                hoodPosition += -gamepad1.left_stick_y/4;
                hoodPosition = clamp(gamepad1.left_stick_y, 0, 1);
                hood.setPosition(hoodPosition);
            }else {
                launcherAngleVelocity();
                hood.setPosition(hoodPosition);
            }
            if (gamepad2.right_trigger >0) {
                setIntakePower(1);
            }else if(gamepad2.left_trigger >0) {
                setIntakePower(-1);
            }else if (!(gamepad1.left_trigger > 0)){
                setIntakePower(0);
            }

          /*  if (gamepad1.x && !x && gamepad1.left_trigger ==0){
               if (index <5) {
                   index += 1;
               } else {
                   index = 0;
               }
               indexer(index);
               x = true;
            } else if (!gamepad1.x) {
                x = false;
            }*/

            if (gamepad1.square) {
                index = 0;
                indexer(index);
            } else if (gamepad1.triangle) {
                index = 2;
                indexer(index);
            } else if (gamepad1.circle) {
                index = 4;
                indexer(index);
            } else {
                indexer(index);
            }

            if (changedTimer.now(TimeUnit.SECONDS) > lastChangedTime + 1) {
                /*if (gamepad1.dpad_left) {
                    changingValue -= 1;
                    if (changingValue < 0) changingValue = 2;
                    lastChangedTime = changedTimer.now(TimeUnit.SECONDS);
                } else if (gamepad1.dpad_right) {
                    changingValue += 1;
                    if (changingValue > 2) changingValue = 0;
                    lastChangedTime = changedTimer.now(TimeUnit.SECONDS);
                }*/
            }
//            if (gamepad1.dpad_up) {
//                power += 100;
//            } else if (gamepad1.dpad_down) {
//                power -= 100;
//            }
            double changeValue = (gamepad1.dpad_up ? 0.01 : 0) + (gamepad1.dpad_down ? -0.01 : 0);
            if (changingValue == 0) {
                Kp += changeValue;
            } else if (changingValue == 1) {
                Ki += changeValue;
            } else if (changingValue == 2) {
                Kd += changeValue;
            }
            if (gamepad1.right_stick_button) {
                if (!launchTriggered) {
                  //  Actions.runBlocking(new launchCycle()/*, new setPowers()*/);
                    new newLaunchCycle(false, true).run(new TelemetryPacket());
                    launchTriggered = true;
                }
                //new launchCycle();
            } else {
                launchTriggered = false;
            }
          //  double TX2 = 0;
            //double TX3 = 0;

            if (result != null && result.isValid()) {
                TX = result.getTx();
                telemetry.addData("Target X", TX);
                // telemetry.addData("Target X2", TX2);
                //telemetry.addData("Target X3", TX3);

              /*  if (gamepad1.cross) {
                    new turretTrack();
                }*/
                // turretYawRobot = result.getBotpose().getOrientation().getYaw(AngleUnit.DEGREES) - drive.localizer.getPose().heading.toDouble();
                //turretYawRobot2 = result.getFiducialResults().get(0).getTargetXDegrees() - drive.localizer.getPose().heading.toDouble();
                // telemetry.addData("Target Y", ty);
                //telemetry.addData("Target Area", ta);


            } else {
                pid.lastError = 0;
                pid.integralSum = 0;
                pid.time = new ElapsedTime();
                telemetry.addData("Limelight", "No Targets");
                //turret2.getController().getServoPosition(2);
               /* if (gamepad1.cross) {
                    turret2.setPower(
                            -pid.PIDControl(Kp, Ki, Kd, Math.atan2(drive.localizer.getPose().position.x - (-58.31), drive.localizer.getPose().position.y - (55.64)), turretYawRobot + drive.localizer.getPose().heading.toDouble())
                    );
                    turret1.setPower(
                            -pid.PIDControl(Kp, Ki, Kd, Math.atan2(drive.localizer.getPose().position.x - (-58.31), drive.localizer.getPose().position.y - (55.64)), turretYawRobot + drive.localizer.getPose().heading.toDouble())
                    );
                }else {
                    turret1.setPower(0);
                }*/

            }
            if (abs(TX) < 4) {
                telemetry.addData("Target:", "Acquired");
            }


                /*   }else {
                turret1.setPower(0);
                turret2.setPower(0);
            }*/
         /*   if (gamepad2.circle) {
                limelight.pipelineSwitch(1);
                telemetry.addData("ID", result.getFiducialResults().get(0).getFiducialId());
            } else {
                limelight.pipelineSwitch(0);
            } */

            drive.setDrivePowers(new PoseVelocity2d(
                    new Vector2d(
                            -gamepad2.left_stick_y,
                            -gamepad2.left_stick_x
                    ),
                    -gamepad2.right_stick_x
            ));

            telemetry.addData("isRedAlliance", isRedAlliance);
            telemetry.addData("isTeleopRed", isTeleOpRed);
            telemetry.addData("teleOpBeginPose", teleOpBeginPose);
            telemetry.addData("light1", indexLightSignal1.getPosition());
            telemetry.addData("light2", indexLightSignal2.getPosition());
            telemetry.addData("light3", indexLightSignal3.getPosition());
            telemetry.addData("localizerPose", drive.localizer.getPose());

            telemetry.addData("teleOpDistancePower", teleopPower);
            telemetry.addData("autoPower", autoPower);
            telemetry.addData("turretPOWER", turret1.getPower());
            telemetry.addData("redTagX", redGoalX);
            telemetry.addData("redTagY", redGoalY);
            telemetry.addData("redTagVector", redGoalVector);
            telemetry.addData("redTagHeading", redGoalHeading);

            telemetry.addData("blueTagVector", blueGoalVector);
            telemetry.addData("blueTagHeading", blueGoalHeading);
            telemetry.addData("Launcher Velocity", launcher.getVelocity(AngleUnit.DEGREES));
          //  telemetry.addData("Launcher Velocity Target", );
            telemetry.addData("Launcher Velocity Adjusted", launcher.getVelocity(AngleUnit.DEGREES)*19.1);
            telemetry.addData("turretRaw", turretFB.getVoltage()/3.3*360/4);

            if (result != null) {
                Pose3D botpose = result.getBotpose();
                Pose3D botposeMT2 = result.getBotpose_MT2();

                telemetry.addData("tx", result.getTx());
                //telemetry.addData("rz", result.getRz());
                telemetry.addData("ty", result.getTy());
                telemetry.addData("ta", result.getTa());
                telemetry.addData("Botpose", botpose.toString());
                telemetry.addData("BotposeMT2", botposeMT2.toString());

            }
            telemetry.addData("artifacts", artifacts);
            telemetry.addData("index", index);
            telemetry.addData("color G", color1.green());
            telemetry.addData("color P", (color1.red()+ color1.blue())/2);
            telemetry.addData("color2 G", color2.green());
            telemetry.addData("color2 P", (color2.red()+ color2.blue())/2);
            telemetry.addData("color P", (color1.red()+color1.blue())/2);
            telemetry.addData("color G2", color2.green());
            telemetry.addData("color P2", (color2.red()+color2.blue())/2);
            telemetry.addData("PPX", drive.localizer.getPose().position.x);
            telemetry.addData("PPY", drive.localizer.getPose().position.y);
            telemetry.addData("PPYaw", Math.toDegrees(drive.localizer.getPose().heading.toDouble()));
              telemetry.addData("calculated angle to red", Math.atan2(drive.localizer.getPose().position.x-(-58.31), drive.localizer.getPose().position.y - (55.64)));
//            telemetry.addData("turretYawRobot", turretYawRobot);
//            telemetry.addData("turretYawField", turretYawRobot+drive.localizer.getPose().heading.toDouble());
//            telemetry.addData("turretYawField2", turretYawRobot2);
              if (result != null) {
                telemetry.addData("X", result.getBotpose().getPosition().x);
                telemetry.addData("Y", result.getBotpose().getPosition().y);
                telemetry.addData("TurretHeading", result.getBotpose().getOrientation().getYaw(AngleUnit.DEGREES));
                telemetry.addData("turretYaw3", result.getTx());
            }
            //telemetry.addData("feedPos", feed.getPosition())
            assert result != null;
            if (!result.getFiducialResults().isEmpty()) {telemetry.addData("fiducial result Tx", result.getFiducialResults().get(0).getTargetXDegrees());}
//            telemetry.addData("TxNC", result.getTxNC());
//            telemetry.addData("Tx", result.getTx());
//            telemetry.addData("Power", launcher.getPower());
//            telemetry.addData("Launch Current", launcher.getCurrent(CurrentUnit.AMPS));
//            telemetry.addData("RPM", launcher.getVelocity(AngleUnit.DEGREES));
//            telemetry.addData("RPM Adjusted", launcher.getVelocity(AngleUnit.DEGREES)*20);
//            telemetry.addData("TargetRPM", gamepad1.right_stick_y*6000);
//            telemetry.addData("Right Stick Y", gamepad1.right_stick_y);
//            telemetry.addData("feedPos", feed.getPosition());
//            telemetry.addData("Left Trigger", gamepad1.left_trigger);
//            telemetry.addData("Current", launcher.getCurrent(CurrentUnit.AMPS));
//            telemetry.addData("Hood Position", hood.getPosition());
//            telemetry.addData("turret1", turret1.getPower());
//            telemetry.addData("turret2", turret2.getPower());
            telemetry.addLine();
            if (changingValue == 0) {
                telemetry.addData("Currently changing", "Kp");
            } else if (changingValue == 1) {
                telemetry.addData("Currently changing", "Ki");
            } else if (changingValue == 2) {
                telemetry.addData("Currently changing", "Kd");
            }
            telemetry.addLine();
            telemetry.addData("Kp", Kp);
            telemetry.addData("Ki", Ki);
            telemetry.addData("Kd", Kd);
           // telemetry.addData("turret1pow", turret1.getController().getPwmStatus());
            telemetry.update();
            TelemetryPacket packet = new TelemetryPacket();
            packet.fieldOverlay().setStroke("#3F51B5");
            packet.put("x", drive.localizer.getPose().position.x);
            packet.put("y", drive.localizer.getPose().position.y);

            Drawing.drawRobot(packet.fieldOverlay(), drive.localizer.getPose());
            FtcDashboard.getInstance().sendTelemetryPacket(packet);

        }
    }
   
    // A timer to calculate the change in time (delta time)
   // ElapsedTime timer = new ElapsedTime();

}
