package org.firstinspires.ftc.teamcode.opmodes.teleop;

import static androidx.core.math.MathUtils.clamp;
import static java.lang.Math.PI;
import static java.lang.Math.abs;

import org.firstinspires.ftc.teamcode.RobotConstants;
import org.firstinspires.ftc.teamcode.robot;
import org.firstinspires.ftc.teamcode.util.Drawing;
import org.firstinspires.ftc.teamcode.util.PID;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
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
        double Kp = RobotConstants.DriveControl.kP;
        double Ki = RobotConstants.DriveControl.kI;
        double Kd = RobotConstants.DriveControl.kD;
        double changingValue = 0;

        boolean x = false;
        boolean launchTriggered = false;
        boolean dpadLeftPressed = false;
        boolean dpadRightPressed = false;
        boolean indexUp = false;
        boolean indexDown = false;
        ElapsedTime time = new ElapsedTime(ElapsedTime.Resolution.SECONDS);

        int lastCheck;
        double checkInterval = RobotConstants.Misc.checkInterval;


        Pose2d beginPose;
        //yaw of turret in robot space, 0 is center, positive is right, negative is left
        double turretYawRobot = 0.0;
        double turretYawRobot2 = 0.0;
        PID pid = new PID();
        double TX = 0;
       // AprilTag aprilTag = new AprilTag("Webcam 1", hardwareMap);
        boolean beginPoseFound = false;
        //Limelight3A limelight = hardwareMap.get(Limelight3A.class, "limelight");

        telemetry.setMsTransmissionInterval(10);

        //imelight.pipelineSwitch(0);
        double gValue;
        double rValue;
        double bValue;

        /*
         * Starts polling for data.  If you neglect to call start(), getLatestResult() will return null.
         */
        //limelight.start();
        drive.localizer.setPose(teleOpBeginPose);

        //  LLResult result = limelight.getLatestResult();
        while (opModeInInit()) {
            if (limelight.isRunning()) {
                result = limelight.getLatestResult();
            }
            if (result.isValid() && result != null) {
                drive.localizer.setPose(new Pose2d(new Vector2d(drive.localizer.getPose().position.x, drive.localizer.getPose().position.y),result.getBotpose().getOrientation().getYaw(AngleUnit.RADIANS)+PI));
                //limelight.updateRobotOrientation(Math.toDegrees(drive.localizer.getPose().heading.toDouble()));
            }
            turret1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
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
        time.reset();
        setTurretPosition(targetTurretAngle);
        while (opModeIsActive()) {
            result = limelight.getLatestResult();
            boolean resultValidNoNull = result.isValid() && result != null;
            double timeNow = time.now(TimeUnit.SECONDS);

            // Inside your loop where you check for Dashboard changes:
            if (RobotConstants.TurretPID.kP != lastP || RobotConstants.TurretPID.kI != lastI ||
                    RobotConstants.TurretPID.kD != lastD || RobotConstants.TurretPID.kF != lastF) {

                // 1. Temporarily change mode to allow coefficient updates
              /*  turret1.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);

                // 2. Set the coefficients
                turret1.setPIDFCoefficients(DcMotorEx.RunMode.RUN_USING_ENCODER,
                        new PIDFCoefficients(
                                RobotConstants.TurretPID.kP,
                                RobotConstants.TurretPID.kI,
                                RobotConstants.TurretPID.kD,
                                RobotConstants.TurretPID.kF
                        )
                );

                // 3. Switch back to RUN_TO_POSITION
                turret1.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
*/
                // 4. Update tracking variables
                lastP = RobotConstants.TurretPID.kP;
                lastI = RobotConstants.TurretPID.kI;
                lastD = RobotConstants.TurretPID.kD;
                lastF = RobotConstants.TurretPID.kF;
            }
            drive.updatePoseEstimate();
            drive.localizer.update();
            if (timeNow > checkInterval && resultValidNoNull) {

                drive.localizer.setPose(updatePoseFromLimeLight());
                time.reset();

            }
            robot.PARAMS.localizerX = drive.localizer.getPose().position.x;
            robot.PARAMS.localizerY = drive.localizer.getPose().position.y;
            blueGoalHeading = Math.toDegrees(Math.atan2(blueGoalY -PARAMS.localizerY, blueGoalX -PARAMS.localizerX));
            blueGoalDistance = Math.hypot(blueGoalX -PARAMS.localizerX, blueGoalY -PARAMS.localizerY);
            redGoalHeading = Math.toDegrees(Math.atan2(redGoalY -PARAMS.localizerY, redGoalX -PARAMS.localizerX));
            redGoalDistance = Math.hypot(redGoalX -PARAMS.localizerX, redGoalY -PARAMS.localizerY);
           // drive.localizer.setPose(result.getBotpose());
            YawPitchRollAngles orientation = imu.getRobotYawPitchRollAngles();
            //limelight.updateRobotOrientation(orientation.getYaw());
            colorLogger();
            indexLoadManagement();

           /* if (gamepad1.dpad_left || gamepad2.dpad_left && !dpadLeftPressed) {
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
            }*/

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

            //MANUAL TURRET CONTROL
            if (gamepad1.right_bumper || gamepad2.triangle && !gamepad2.cross) {
                targetTurretAngle -= gamepad1.right_stick_x * 8;
                targetTurretAngle -= gamepad2.right_stick_x * 8;
                //turretPID.turretReset();
                //PARAMS.turretAngle = 180;
            } else if (gamepad1.share || gamepad2.share || !isPoseUpdatedFromStart){ //TURRET TO ZERO POSITION
                targetTurretAngle = 0;
            } else { //DEFAULT: AUTOMATIC TURRET CONTROL
            updateTurretTracking(isRedAlliance);
            }
            if (gamepad2.triangle && gamepad2.cross){
                Actions.runBlocking(new SequentialAction(drive.actionBuilder(drive.localizer.getPose())
                        .splineToSplineHeading(new Pose2d(PARAMS.parkX, PARAMS.parkY, drive.localizer.getPose().heading.toDouble()), Math.toRadians(180))
                        .build()
                ));
            }

            // Always actuate turret every loop
            setTurretPosition(targetTurretAngle);

            if (resultValidNoNull) {
                telemetry.addData("Target Locked?", true);

            }

            if (gamepad1.dpad_up){
                liftL.setPower(1);

            } else if (gamepad1.dpad_down) {
                liftL.setPower(-1);

            } else {
                liftL.setPower(0);
            }
            if (gamepad1.right_trigger > 0.0) {

                setLaunchRPM(1500);
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
                hoodPosition += -gamepad1.left_stick_y/10;
                //hoodPosition = clamp(hoodPosition, 0.5, 1);
                hood.setPosition(hoodPosition);
                teleopPower = 5000;
                if (gamepad2.right_bumper) {
                    indexer.setPower(1);
                } else {indexer.setPower(0);}
            }else {
                launcherAngleVelocity();
                hood.setPosition(hoodPosition);
            }
            if (gamepad2.right_trigger >0 && gamepad2.left_trigger < 0.1) {
                setIntakePower(1);
            }else if(gamepad2.left_trigger >0 && gamepad2.right_trigger < 0.1) {
                setIntakePower(-1);
            }else if (!(gamepad1.left_trigger > 0)){
                setIntakePower(0);
            }
            if (gamepad2.right_trigger > 0.5 && gamepad2.left_trigger > 0.5) {
                drive.setDrivePowers(new PoseVelocity2d(
                        new Vector2d(
                                -0.5*gamepad2.left_stick_y,
                                -0.5*gamepad2.left_stick_x
                        ),
                        -0.5*gamepad2.right_stick_x
                ));
            } else {
                drive.setDrivePowers(new PoseVelocity2d(
                        new Vector2d(
                                -gamepad2.left_stick_y,
                                -gamepad2.left_stick_x
                        ),
                        -gamepad2.right_stick_x
                ));
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

            if (gamepad2.circle) {
                freeIntake();

            }

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

            //TRIGGER AUTOMATIC LAUNCH CYCLE
            if (gamepad1.right_stick_button || gamepad2.right_bumper && !gamepad1.left_bumper && !gamepad2.left_bumper) {
                if (!launchTriggered) {
                  //  Actions.runBlocking(new launchCycle()/*, new setPowers()*/);
                    new newLaunchCycle(false, false).run(new TelemetryPacket());
                    launchTriggered = true;
                }
                //new launchCycle();
            } else {
                launchTriggered = false;
            }
          //  double TX2 = 0;
            //double TX3 = 0;


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
            telemetry.addData("redTagVector", RobotConstants.FieldPositions.redGoalVector);
            telemetry.addData("redTagHeading", redGoalHeading);

            telemetry.addData("blueTagVector", RobotConstants.FieldPositions.blueGoalVector);
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
            packet.put("Result valid and not null:", resultValidNoNull);
            packet.put("Ta", result.getTa());
            packet.put("Hood", hoodPosition);
            packet.put("Heading Confirmed:", isHeadingConfirmed);
            packet.put("turretVelocity", Math.abs(turret1.getVelocity(AngleUnit.DEGREES)));
            packet.put("x", drive.localizer.getPose().position.x);
            packet.put("y", drive.localizer.getPose().position.y);
            packet.put("Turret Target", getTargetTurretPosition());
            packet.put("Turret Actual", getTurretPosition());
            packet.put("launchRPM", launcher.getVelocity(AngleUnit.DEGREES) * 19.1);
            packet.put("turretPow", turret1.getVelocity(AngleUnit.DEGREES));
            packet.put("IMU", imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES));
            packet.put("heading", Math.toDegrees(drive.localizer.getPose().heading.toDouble()));
            if (limeLightBotpose != null) {
                packet.put("LLX", limeLightBotpose.position.x);
                packet.put("LLY", limeLightBotpose.position.y);
                packet.put("LLheading", Math.toDegrees(limeLightBotpose.heading.toDouble()));
            }
            Drawing.drawRobot(packet.fieldOverlay(), drive.localizer.getPose());
            FtcDashboard.getInstance().sendTelemetryPacket(packet);

        }
    }
   
    // A timer to calculate the change in time (delta time)
   // ElapsedTime timer = new ElapsedTime();

}
