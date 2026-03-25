package org.firstinspires.ftc.teamcode;

/* Copyright (c) 2025 FIRST. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of FIRST nor the names of its contributors may be used to endorse or
 * promote products derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */


import static androidx.core.math.MathUtils.clamp;
import static java.lang.Math.abs;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
//import com.qualcomm.ftcrobotcontroller.
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.SleepAction;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.util.AprilTag;
import org.firstinspires.ftc.teamcode.util.PID;
import org.firstinspires.ftc.teamcode.subsystems.MecanumDrive;
import org.firstinspires.ftc.teamcode.subsystems.PinpointLocalizer;
import org.firstinspires.ftc.vision.apriltag.AprilTagGameDatabase;
import org.firstinspires.ftc.vision.apriltag.AprilTagLibrary;
import org.slf4j.Logger;
import org.slf4j.LoggerFactory;

import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import java.util.Arrays;
import java.util.List;
import java.util.concurrent.TimeUnit;

@TeleOp(name = "Robot", group = "Robot")
@Disabled
public class robot extends LinearOpMode {

    private static final Logger log = LoggerFactory.getLogger(robot.class);
    protected AprilTag aprilTag;
    protected AnalogInput indexFB;
    protected AnalogInput turretFB;
    protected DcMotorEx launcher;
    protected DcMotorEx liftL;
    protected DcMotorEx liftR;
    protected double turnVector;
    protected DcMotorEx intake;
    protected CRServo launchFeedL;
    protected CRServo launchFeedR;
    protected CRServo indexer;
    public CRServo turret1;
    protected Servo hood;
    protected Servo feed;
    protected ColorSensor color1;
    protected ColorSensor color2;

    protected Servo indexLightSignal1;
    protected Servo indexLightSignal2;
    protected Servo indexLightSignal3;

    public MecanumDrive drive;

    public double TICKS_PER_REV = RobotConstants.Misc.ticksPerRev;

    protected int lastPosLeft;
    protected int lastPosRight;
    protected long lastTime;
    protected int index = 0;
    protected int power = 3400;
    protected boolean isLaunching;
    protected PinpointLocalizer PinpointLocalizer;
    protected IMU imu;
    protected double lastIndexPosition;
    protected double totalIndexRotation = 0;
    protected int totalRotations = 0;
    protected double lastTurretPosition = 0;
    protected double totalTurretRotation = 0;
    protected int totalTurretRotations = 0;
    protected double lastTargetAngle = 0;
    public int modifier = 1;
    protected Pose2d limeLightBotpose;

    protected double conversionRatio = RobotConstants.Misc.conversionRatio;

    int rotationTicker = 0;
    int lastIndex = 0;
    boolean indexCC = true;
    // Runtime state (computed per-alliance, not tunable constants)
    public static class Params {
        public double beginPosX = RobotConstants.StartPositions.beginPosX;
        public double beginPosY = RobotConstants.StartPositions.beginPosY;
        public double localizerX = beginPosX;
        public double localizerY = beginPosY;
        public double beginHeading = RobotConstants.StartPositions.beginHeading;
        public double targetX = RobotConstants.AutoPaths.targetX;
        public double targetY = RobotConstants.AutoPaths.targetY;
        public double row1X = RobotConstants.AutoPaths.row1X;
        public double row1Y = RobotConstants.AutoPaths.row1Y;
        public double row2X = RobotConstants.AutoPaths.row2X;
        public double row2Y = RobotConstants.AutoPaths.row2Y;
        public double row3X = RobotConstants.AutoPaths.row3X;
        public double row3Y = RobotConstants.AutoPaths.row3Y;
        public double backToY = RobotConstants.AutoPaths.backToY;
        public double row1CheckY = row1Y;
        public double row2CheckY = row2Y;
        public double row3CheckY = row3Y;
        public double intakeHeading = Math.toRadians(90);
        public double endX = 30;
        public double endY = -20;
        public double targetHeading = RobotConstants.AutoPaths.targetHeading;
        public double turretP = RobotConstants.TurretPID.kP;
        public double turretI = RobotConstants.TurretPID.kI;
        public double turretD = RobotConstants.TurretPID.kD;
        public double turretZeroOffset = RobotConstants.TurretPID.zeroOffset;
        public double indexerP = RobotConstants.IndexerPID.kP;
        public double indexerI = RobotConstants.IndexerPID.kI;
        public double indexerD = RobotConstants.IndexerPID.kD;
        public int indexCycleStart = 0;
    }
    public boolean isRedAlliance = false;
    public static Params PARAMS = new Params();
    public double targetTurretAngle = 0;
    public int teleopPower = RobotConstants.LauncherCalibration.teleopPower;
    public int autoPower = RobotConstants.LauncherCalibration.autoPower;
    public double hoodPosition = 0;
    public Pose2d beginPos = new Pose2d(new Vector2d(PARAMS.beginPosX, PARAMS.beginPosY), PARAMS.beginHeading);
    public static Pose2d teleOpBeginPose;

    public Pose2d globalLoc() {
        double x = PinpointLocalizer.getPose().position.x;
        double y = PinpointLocalizer.getPose().position.y;
        double theta = PinpointLocalizer.getPose().heading.toDouble();
        return new Pose2d(x, y, theta);
    }
    protected PID pid;
    protected PID indexPID;
    protected PID turretPID;
    protected Limelight3A limelight;
    protected LLResult result;
    public AprilTagLibrary tagLibrary = AprilTagGameDatabase.getDecodeTagLibrary();

    public AprilTagLibrary getTagLibrary() {
        return tagLibrary;
    }


    protected double redGoalX = RobotConstants.FieldPositions.redGoalX;
    protected double redGoalY = RobotConstants.FieldPositions.redGoalY;
    protected double redGoalHeading = 0;
    protected double redGoalDistance = 0;

    protected double blueGoalX = RobotConstants.FieldPositions.blueGoalX;
    protected double blueGoalY = RobotConstants.FieldPositions.blueGoalY;
    protected double blueGoalHeading = 0;
    protected double blueGoalDistance = 0;

    protected List<Integer> artifacts;
    @Override
    public void runOpMode() throws InterruptedException {

        if (isRedAlliance){
            modifier = 1;
        }else {modifier = -1;}
        //public double beginPosX = 65;
        PARAMS.beginPosY = 11*modifier;
       // public double localizerX = beginPosX;
        //public double localizerY = beginPosY;
        PARAMS.beginHeading = Math.toRadians(180)*modifier;
        //public double targetX = -53;
        PARAMS.targetY = 25*modifier;
        //public double row1X = 34.5;
        PARAMS.row1Y = 68*modifier;
        //row2X = 10;
        PARAMS.row2Y = 69*modifier;
       // public double row3X = -13;
        PARAMS.row3Y = 62*modifier;
        PARAMS.backToY = 25*modifier;
        PARAMS.row1CheckY = PARAMS.row1Y - (1*modifier);
        PARAMS.row2CheckY = PARAMS.row2Y - (1*modifier);
        PARAMS.row3CheckY = PARAMS.row3Y - (1*modifier);
        PARAMS.intakeHeading = Math.toRadians(90)*modifier;
       // public double endX = 30;
        //public double endY = -20;
        PARAMS.targetHeading = 110 * modifier;

        beginPos = new Pose2d(new Vector2d(PARAMS.beginPosX, PARAMS.beginPosY), PARAMS.beginHeading);
        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        result = limelight.getLatestResult();
        //pid = new PID();
        hood = hardwareMap.get(Servo.class, "hood");
        indexFB = hardwareMap.get(AnalogInput.class, "indexFB");
        // Servo feed = hardwareMap.get(Servo.class, "feed");
        intake = hardwareMap.get(DcMotorEx.class, "intake");
        turret1 = hardwareMap.get(CRServo.class, "turret1");
        turretFB = hardwareMap.get(AnalogInput.class, "turretFB");
        //turret2 = hardwareMap.get(CRServo.class, "turret2");
       // feed = hardwareMap.get(Servo.class, "feed");
        launchFeedL = hardwareMap.get(CRServo.class, "launchFeedL");
        launchFeedR = hardwareMap.get(CRServo.class, "launchFeedR");
        launcher = hardwareMap.get(DcMotorEx.class, "launcher");
        indexer = hardwareMap.get(CRServo.class, "index");
        color1 = hardwareMap.get(ColorSensor.class, "color1");
        color2 = hardwareMap.get(ColorSensor.class, "color2");
        liftL = hardwareMap.get(DcMotorEx.class, "liftL");
        liftR = hardwareMap.get(DcMotorEx.class, "liftR");

        indexLightSignal1 = hardwareMap.get(Servo.class, "light1");
        indexLightSignal2 = hardwareMap.get(Servo.class, "light2");
        indexLightSignal3 = hardwareMap.get(Servo.class, "light3");

        //  turret1.scaleRange(0.25, .75);
        //turret2.scaleRange(.25, 0.75);
       // indexer.scaleRange(0, 1);
        // turret1.getController().
        turret1.setDirection(CRServo.Direction.FORWARD);
        turret1.setPower(0);

        indexLightSignal1 = hardwareMap.get(Servo.class, "light1");
        indexLightSignal2 = hardwareMap.get(Servo.class, "light2");
        indexLightSignal3 = hardwareMap.get(Servo.class, "light3");


       // turret2.setDirection(CRServo.Direction.REVERSE);
        PinpointLocalizer = new PinpointLocalizer(hardwareMap, 0.00072471557, new Pose2d(0, 0, 0));
        PinpointLocalizer.driver.resetPosAndIMU();
        pid = new PID();
        indexPID = new PID();
        turretPID = new PID();
        launcher.setVelocityPIDFCoefficients(
                RobotConstants.LauncherPID.kP, RobotConstants.LauncherPID.kI,
                RobotConstants.LauncherPID.kD, RobotConstants.LauncherPID.kF);
      //  launchFeed.setDirection(DcMotorSimple.Direction.FORWARD);

        imu = hardwareMap.get(IMU.class, "imu");
        lastIndexPosition = (indexFB.getVoltage() / 3.3) * 360.0;
        artifacts = Arrays.asList(0, 0, 0);
        //0 = empty
        //1 = purple
        //2 = green
        //limelight.

        RevHubOrientationOnRobot orientationOnRobot = new
                RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.LEFT,
                RevHubOrientationOnRobot.UsbFacingDirection.UP
        );

       // imu.initialize(new IMU.Parameters(orientationOnRobot));
       // lastPosLeft = launcherLeft.getCurrentPosition();
        //lastPosRight = launcherRight.getCurrentPosition();
        lastTime = System.nanoTime();
        // TODO: Make the start values based on the april tags

        double startX = 0;
        double startY = 0;
        double startTheta = 0;
        //PinpointLocalizer.setPose(PARAMS.beginPos);
        imu.initialize(new IMU.Parameters(orientationOnRobot));
        limelight.start();
        if (result != null && result.isValid()) {
           // beginPos = new Pose2d(new Vector2d(result.getBotpose().getPosition().x*39.3701, result.getBotpose().getPosition().y*39.3701), Math.toRadians(result.getBotpose().getOrientation().getYaw()));
        }
        telemetry.addData("beginPosition", beginPos);
        telemetry.addData("LLValid", result.isValid());
        telemetry.update();
        //aprilTag = new AprilTag("Webcam 1", hardwareMap);
        waitForStart();
        drive = new MecanumDrive(hardwareMap, beginPos);
        drive.localizer.update();
        blueGoalHeading = Math.atan2(blueGoalY -PARAMS.localizerY, blueGoalX -PARAMS.localizerX);
        blueGoalDistance = Math.hypot(blueGoalX -PARAMS.localizerX, blueGoalY -PARAMS.localizerY);
        redGoalHeading = Math.atan2(redGoalY -PARAMS.localizerY, redGoalX -PARAMS.localizerX);
        redGoalDistance = Math.hypot(redGoalX -PARAMS.localizerX, redGoalY -PARAMS.localizerY);
        // aprilTag = new AprilTag("Webcam 1", hardwareMap);

        if (isStopRequested()){
            indexer(0);
        }
    }

    public double getTurretPosition() {
        double currentAngle = ((turretFB.getVoltage() / 3.3) * 360.0);
        if (lastTurretPosition - currentAngle >= 280) {
            totalTurretRotations += 1;
        } else if (lastTurretPosition - currentAngle <= -280) {
            totalTurretRotations -= 1;
        }
        totalTurretRotation = (currentAngle + totalTurretRotations * 360) / 4.167;
        lastTurretPosition = currentAngle;
        return totalTurretRotation;
    }

    public boolean updatePoseFromLimeLight () {
        double turretPos = getTurretPosition();
        double turretTarget = targetTurretAngle + PARAMS.turretZeroOffset;
        if (result != null && result.isValid() && abs(turretTarget - turretPos) < 3) {
            limeLightBotpose = new Pose2d(new Vector2d(result.getBotpose().getPosition().x*conversionRatio, result.getBotpose().getPosition().y*conversionRatio), result.getBotpose().getOrientation().getYaw(AngleUnit.RADIANS));
            drive.localizer.setPose(limeLightBotpose);
            indexLightSignal2.setPosition(1);
            return true;
        } else {
            return false;
        }
    }

    // RETURNS INDEX OF BALL WITH SPECIFIED COLOR, OR -1 IF NOT FOUND
    //This is great, thanks!
    public int findColorIndex(int artifactColor) {
        for (int i = 0; i < artifacts.toArray().length; i++) {
            int artifact = artifacts.get(i);
            if (artifact == artifactColor) return i;
        }
        return -1;
    }
    //Updates the teleOpBeginPose at the end of auto
    public class sendAutoEndPose implements Action{
        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            drive.localizer.update();
            teleOpBeginPose = drive.localizer.getPose();
            return false;
        }
    }
    //Sets the hood position and launch velocity based on distance to goal
    public void launcherAngleVelocity(){
        double tagDistance = isRedAlliance ? redGoalDistance : blueGoalDistance;

        hoodPosition = (RobotConstants.LauncherCalibration.hoodM * tagDistance) + RobotConstants.LauncherCalibration.hoodB;
        teleopPower = Math.toIntExact(Math.round(
                (RobotConstants.LauncherCalibration.launcherM * tagDistance) + RobotConstants.LauncherCalibration.launcherB));

        telemetry.addData("tagDistance", tagDistance);
        telemetry.addData("hoodPosition", hoodPosition);
        telemetry.addData("observedHoodPosition", hood.getPosition());
        telemetry.addData("teleopPower", teleopPower);
        hood.setPosition(clamp(hoodPosition, RobotConstants.LauncherCalibration.hoodMin, RobotConstants.LauncherCalibration.hoodMax));
    }
    //Sets turret position
    public double setTurretPosition(double targetPosition) {
        double currentAngle = ((turretFB.getVoltage() / 3.3) * 360.0);
        double turretTargetAngle = targetPosition + PARAMS.turretZeroOffset;
        double power;
        if (lastTurretPosition - currentAngle >= 280) {
            totalTurretRotations += 1;
        } else if (lastTurretPosition - currentAngle <= -280) {
            totalTurretRotations -= 1;
        }
        totalTurretRotation = (currentAngle + totalTurretRotations * 360) / 4.167;

        // Use dedicated turretPID instance (not general pid)
        power = -turretPID.turretPID(PARAMS.turretP, PARAMS.turretI, PARAMS.turretD, turretTargetAngle, totalTurretRotation);

        turret1.setPower(clamp(power, -1, 1));
        lastTurretPosition = currentAngle;

        telemetry.addData("turretServoPower", turret1.getPower());
        telemetry.addData("Turret Target", turretTargetAngle);
        telemetry.addData("Turret Current", totalTurretRotation);
        telemetry.addData("Turret Power", power);
        telemetry.addData("Turret Rotations", totalTurretRotations);
        return totalTurretRotation;

    }
    //sets index position
    public void setIndexPosition(double targetPosition) {
       // indexer.setPosition(targetPosition);
      //  double targetPosition
        double currentAngle = (indexFB.getVoltage() / 3.3) * 360.0;
       // totalIndexRotation = currentAngle;
        double indexerTargetAngle = targetPosition*360;
       /* if (indexerTargetAngle == 0 && currentAngle > 260) {
            indexerTargetAngle = 360;
        }*/

        if (lastIndexPosition-currentAngle >=280){
            totalRotations += 1;
        } else if (lastIndexPosition - currentAngle <= -280) {
            totalRotations -=1;
        }
        totalIndexRotation = currentAngle + totalRotations*360;
        double totalTargetAngle = indexerTargetAngle + totalRotations*360;
        double power;
        power = -pid.indexPID(PARAMS.indexerP, PARAMS.indexerI, PARAMS.indexerD, indexerTargetAngle + 360*rotationTicker, totalIndexRotation);

        // Apply a deadband to prevent the servo from whining when it's at the target
        if (abs(indexerTargetAngle - currentAngle) < 2.0) { // 2-degree tolerance
            power = 0;
        }

        // Set the CR Servo power
        indexer.setPower(clamp(10*power, -0.5, 0.5));

        lastIndexPosition = currentAngle;
       /* telemetry.addData("Indexer Target", indexerTargetAngle);
        telemetry.addData("Indexer Current", currentAngle);
        telemetry.addData("Indexer Total", totalIndexRotation);
        telemetry.addData("Indexer Target Adjusted", indexerTargetAngle + 360*totalRotations);
        telemetry.addData("Indexer Power", power);
        telemetry.addData("Indexer Rotations", totalRotations);
        telemetry.addData("Rotation Ticker", rotationTicker);*/


    }
    // --- ROBOT)
    //Not in use
    public void driveFieldRelative(double forward, double right, double rotate) {
        double theta = Math.atan2(forward, right);
        double r = Math.hypot(right, forward);

        theta = AngleUnit.normalizeRadians(
                theta - PinpointLocalizer.getPose().heading.toDouble() * Math.PI / 180
        );

        double newForward = r * Math.sin(theta);
        double newRight = r * Math.cos(theta);

        drive.setDrivePowers(new PoseVelocity2d(
                new Vector2d(
                        -newForward,
                        -newRight
                ),
                -theta
        ));

    }

    public void setIntakePower(double power) {
        intake.setPower(power);
    }
    public class autoIntakeOn implements Action {
        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            intake.setPower(1);
            indexer.setPower(-0.6);
            return false;
        }
    }
    public class autoIntakeOff implements Action {
        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            intake.setPower(0);
            indexer.setPower(0);
            return false;
        }
    }
    public class rowSelectAuto implements Action {
        public int row;
        public rowSelectAuto(int row) {
            this.row = row;
        }
        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            switch (row) {
                //case 1-3 = row to target
                //case 0 = get out of way
                //case 4 = preload
                //case anything else = do nothing
                case 0:
                    Actions.runBlocking(new SequentialAction(
                            drive.actionBuilder(drive.localizer.getPose())
                                    .strafeToLinearHeading(new Vector2d(PARAMS.row1X, PARAMS.backToY), Math.toRadians(180))
                                    .build()
                    ));
                case 1:

                    Actions.runBlocking(new SequentialAction(
                            drive.actionBuilder(drive.localizer.getPose())
                                    .splineToSplineHeading(new Pose2d(PARAMS.row1X, PARAMS.targetY, Math.toRadians(90)), Math.toRadians(90))
                                    .build(),
                            new autoIntakeOn(),
                            drive.actionBuilder(new Pose2d(PARAMS.row1X, PARAMS.targetY, Math.toRadians(90)))
                                    .splineToSplineHeading(new Pose2d(PARAMS.row1X, PARAMS.row1Y, Math.toRadians(90)), Math.toRadians(90))
                                    .waitSeconds(0.5)
                                    .splineToSplineHeading(new Pose2d(PARAMS.row1X+5, PARAMS.targetY+5, Math.toRadians(180)), Math.toRadians(0))
                                    .splineToSplineHeading(new Pose2d(PARAMS.targetX, PARAMS.targetY, Math.toRadians(PARAMS.targetHeading)), Math.toRadians(175))
                                    .build(),
                            new autoIntakeOff()
                    ));
                    break;
                case 2:
                    Actions.runBlocking(new SequentialAction(
                            drive.actionBuilder(drive.localizer.getPose())
                                    .splineToSplineHeading(new Pose2d(PARAMS.row2X, PARAMS.targetY, PARAMS.intakeHeading), PARAMS.intakeHeading)

                                    //.strafeToLinearHeading(new Vector2d(PARAMS.row2X, PARAMS.targetY), Math.toRadians(90))
                                    .build(),
                            new autoIntakeOn(),
                            drive.actionBuilder(new Pose2d(PARAMS.row2X, PARAMS.targetY,PARAMS.intakeHeading))
                                    .strafeToLinearHeading(new Vector2d(PARAMS.row2X, PARAMS.row2Y), PARAMS.intakeHeading)
                                    .lineToY(PARAMS.row2CheckY)
                                    .lineToY(PARAMS.row2Y)
                                    .waitSeconds(0.2)
                                    .strafeToLinearHeading(new Vector2d(PARAMS.row2X, PARAMS.backToY), PARAMS.intakeHeading)
                                    //.splineToSplineHeading(new Pose2d(PARAMS.row2X+5, PARAMS.targetY + 5, Math.toRadians(180)), Math.toRadians(180))
                                    .strafeToLinearHeading(new Vector2d((PARAMS.targetX + 6), PARAMS.targetY - (5* modifier)), Math.toRadians(PARAMS.targetHeading))
                                    //.splineToSplineHeading(new Pose2d(PARAMS.targetX, PARAMS.targetY, Math.toRadians(PARAMS.targetHeading)), Math.toRadians(175))
                                    .build(),
                            new autoIntakeOff()
                    ));
                    break;
                case 3:
                    Actions.runBlocking(new SequentialAction(
                            drive.actionBuilder(drive.localizer.getPose())
                                   // .splineToSplineHeading(new Pose2d(PARAMS.row3X, PARAMS.targetY, Math.toRadians(90)), Math.toRadians(90))
                                    .strafeToLinearHeading(new Vector2d(PARAMS.row3X, PARAMS.targetY), PARAMS.intakeHeading)
                                    .build(),
                            new autoIntakeOn(),
                            drive.actionBuilder(new Pose2d(PARAMS.row3X, PARAMS.targetY, PARAMS.intakeHeading))
//.splineToSplineHeading(new Pose2d(PARAMS.row3X, PARAMS.row3Y, Math.toRadians(90)), Math.toRadians(90))
                                    .strafeToLinearHeading(new Vector2d(PARAMS.row3X, PARAMS.row3Y), PARAMS.intakeHeading)

                                    .lineToY(PARAMS.row3CheckY)
                                    .lineToY(PARAMS.row3Y)
                                  //  .waitSeconds(0.5)
                                  //  .splineToSplineHeading(new Pose2d(PARAMS.row3X, PARAMS.targetY + 5, Math.toRadians(90)), Math.toRadians(90))
                                    .strafeToLinearHeading(new Vector2d((PARAMS.targetX + 4), (PARAMS.targetY - (4* modifier))), Math.toRadians(PARAMS.targetHeading))
                                   // .splineToSplineHeading(new Pose2d(PARAMS.targetX, PARAMS.targetY, Math.toRadians(PARAMS.targetHeading)), Math.toRadians(175))
                                    .build(),
                            new autoIntakeOff()
                    ));
                    break;
                case 4:
                    Actions.runBlocking(new SequentialAction(
                            drive.actionBuilder(beginPos)
                                    .strafeToLinearHeading(new Vector2d(PARAMS.targetX, PARAMS.targetY), Math.toRadians(PARAMS.targetHeading))
                                    .build()
                    ));
                    break;
                case 5:
                    Actions.runBlocking(new SleepAction(13));
                    break;
            }
            return false;
        }
    }

    // --- BUILT-IN PIDF VELOCITY CONTROL ---
    public void setLaunchRPM(double launchRPM) {
        launcher.setVelocity(-launchRPM/19.1, AngleUnit.DEGREES);
        telemetry.addData("Launcher Target Velocity", launchRPM);
        telemetry.addData("Launcher Target Velocity2", -launchRPM/19.1);
       /* if (launchRPM >10) {
            isLaunching = true;
        } else {
            isLaunching = false;
        }*/

    }
    public double getLaunchRPM() {
        return launcher.getVelocity(AngleUnit.DEGREES)*19.1;
    }

    public double rpmToTicksPerSec(double rpm) {
        return (rpm * TICKS_PER_REV) / 60.0;
    }
    public void indexLoadManagement() {
        boolean loaded = artifacts.get(0) + artifacts.get(1) + artifacts.get(2) > 0;
        PARAMS.indexerP = RobotConstants.IndexerPID.kP;
        PARAMS.indexerI = RobotConstants.IndexerPID.kI;
        PARAMS.indexerD = loaded ? RobotConstants.IndexerPID.kD_loaded : RobotConstants.IndexerPID.kD_unloaded;
    }
    public void indexer(int position) {
        if (position < lastIndex) {
            rotationTicker += 1;
        }
        pid.indexReset();
        switch (position) {
            case 1: setIndexPosition(RobotConstants.IndexPositions.pos1); break;
            case 2: setIndexPosition(RobotConstants.IndexPositions.pos2); break;
            case 3: setIndexPosition(RobotConstants.IndexPositions.pos3); break;
            case 4: setIndexPosition(RobotConstants.IndexPositions.pos4); break;
            case 5: setIndexPosition(RobotConstants.IndexPositions.pos5); break;
            case 0:
            default: setIndexPosition(RobotConstants.IndexPositions.pos0); break;
        }
        lastIndex = position;
    }
    // Computes turret angle to track the goal, updates goal heading/distance
    public void updateTurretTracking(boolean redAlliance) {
        turretPID.turretReset();
        drive.localizer.update();
        double locX = drive.localizer.getPose().position.x;
        double locY = drive.localizer.getPose().position.y;
        double robotHeading = drive.localizer.getPose().heading.toDouble();

        double goalX = redAlliance ? redGoalX : blueGoalX;
        double goalY = redAlliance ? redGoalY : blueGoalY;

        // Field-frame angle from robot to goal (radians)
        double fieldAngle = Math.atan2(goalY - locY, goalX - locX);

        // Robot-relative turret angle (degrees)
        targetTurretAngle = Math.toDegrees(
                Math.atan2(Math.sin(fieldAngle - robotHeading), Math.cos(fieldAngle - robotHeading))
        );

        // Update cached distance for hood/velocity calculations
        double dist = Math.hypot(goalX - locX, goalY - locY);
        if (redAlliance) {
            redGoalHeading = Math.toDegrees(fieldAngle);
            redGoalDistance = dist;
        } else {
            blueGoalHeading = Math.toDegrees(fieldAngle);
            blueGoalDistance = dist;
        }

        telemetry.addData("turretTrackAngle", targetTurretAngle);
        telemetry.addData("goalDistance", dist);
    }

    // Action wrapper for auto sequences
    public class turretTrackAction implements Action {
        public boolean redAlliance;
        public turretTrackAction(boolean redAlliance) {
            this.redAlliance = redAlliance;
        }
        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            updateTurretTracking(redAlliance);
            return false;
        }
    }
    public void feedOn() {

        intake.setPower(1);


    }
    public void feedOff() {

        intake.setPower(0);
    }
    public void lightUpdate(int sel, int color) {
        int lightSel;
        //lightSel = (sel+index)%3;
        Servo targetLight;
        //lightSel += (2-index)+2;
       /* if (lightSel <0) {
            lightSel = 3-lightSel;
        }*/

        //lightSel = lightSel%3;
        switch (index/2){
            case 0 :
                lightSel = sel%3;
                break;
            case 1:
                lightSel = (sel+2)%3;
                break;
            case 2:
                lightSel = (sel+1)%3;
                break;
            default:
                lightSel = sel;
                break;

        }
        switch (lightSel) {
            case 1:
                targetLight = indexLightSignal1;
                break;
            case 2:
                targetLight = indexLightSignal3;
                break;
            case 0:
                targetLight = indexLightSignal2;
                break;
            default:
                targetLight = indexLightSignal1;
                break;
        }
        switch (color){
            case 0:
                targetLight.setPosition(0.28);
                break;
            case 1:
                targetLight.setPosition(0.69);
                break;
            case 2:
                targetLight.setPosition(0.51);
                break;
            default:
                targetLight.setPosition(0.28);
                break;
        }

    }
    public void colorLogger() {
        int intakeSlotContents;
        int offset = -1;
        //0 = empty
        //1 = purple
        //2 = green
        double purple1 = (color1.red()+color1.blue())/2;
        double green1 = color1.green();
        double purple2 = (color2.red()+color2.blue())/2;
        double green2 = color2.green();

        double minIntensity = RobotConstants.ColorThresholds.minIntensity;
        double minDiff = RobotConstants.ColorThresholds.minDifference;
        boolean isPurp1 = purple1 > minIntensity && purple1 - green1 > minDiff;
        boolean isGreen1 = green1 > minIntensity && green1 - purple1 > minDiff;
        boolean isPurp2 = purple2 > minIntensity && purple2 - green2 > minDiff;
        boolean isGreen2 = green2 > minIntensity && green2 - purple2 > minDiff;
        if (isGreen1||isGreen2) {
            intakeSlotContents = 2;
        } else if (isPurp1|isPurp2) {
            intakeSlotContents = 1;
        } else {
            intakeSlotContents = 0;
        }
        lightUpdate(0, artifacts.get(0));
        lightUpdate(1, artifacts.get(1));
        lightUpdate(2, artifacts.get(2));

        isLaunching = abs(launcher.getVelocity()) > 10;
        if (isLaunching || intake.getPower() >0.1 || intake.getPower() < -0.1) {
            if (indexer.getPower() < 0.1) {
                switch (index) {
                    case 1:
                        if (isLaunching) {
                            artifacts.set(0, 0);
                        }
                        break;
                    case 2:
                        artifacts.set(1, intakeSlotContents);
                        break;
                    case 3:
                        if (isLaunching) {
                            artifacts.set(1, 0);
                        }
                        break;
                    case 4:
                        artifacts.set(2, intakeSlotContents);
                        break;
                    case 5:
                        if (isLaunching) {
                            artifacts.set(2, 0);
                        }
                        break;
                    case 0:
                        artifacts.set(0, intakeSlotContents);
                        break;
                }
            }
        }
        lightUpdate(0, artifacts.get(0));
        lightUpdate(1, artifacts.get(1));
        lightUpdate(2, artifacts.get(2));
        //telemetry.addData("artifacts", artifacts);
    }

    public class newLaunchCycle implements  Action{
        public boolean auto;
        public boolean shortRun;

        public newLaunchCycle(boolean auto, boolean shortRun) {
            this.auto = auto;
            this.shortRun = shortRun;
        }
        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            ElapsedTime time = new ElapsedTime(ElapsedTime.Resolution.SECONDS);
            double timer = time.now(TimeUnit.SECONDS);

            double startTime = 10;
            double longDelay;
            double shortDelay;

            boolean launchStarted = false;
            feedOn();
            intake.setPower(1);
            indexer.setPower(0);
            if(auto){
                setLaunchRPM(autoPower);
            }

            while (opModeIsActive()) {


                if (!auto) {
                    // Auto-track by default, manual fine-adjust with right_bumper
                    if (gamepad1.right_bumper) {
                        targetTurretAngle -= gamepad1.right_stick_x * 5;
                    } else {
                        updateTurretTracking(isRedAlliance);
                    }
                    launcherAngleVelocity();
                    power = teleopPower;
                    setLaunchRPM(power);
                    setTurretPosition(targetTurretAngle);
                    drive.localizer.update();
                    robot.PARAMS.localizerX = drive.localizer.getPose().position.x;
                    robot.PARAMS.localizerY = drive.localizer.getPose().position.y;
                }
                boolean toSpeed = abs(abs(power) - abs(getLaunchRPM())) <100;

               // colorLogger();
                if(toSpeed&&!launchStarted) {
                    startTime=time.now(TimeUnit.SECONDS)-timer;
                    launchStarted = true;
                }
                if (result.isValid() && 0==1) {
                    limelight.updateRobotOrientation(drive.localizer.getPose().heading.toDouble());
                    drive.localizer.setPose(new Pose2d(new Vector2d(result.getBotpose().getPosition().x/0.0254, result.getBotpose().getPosition().y/0.0254), result.getBotpose().getOrientation().getYaw()));

                    telemetry.addData("x", drive.localizer.getPose().position.x/0.0254);
                    telemetry.addData("y", drive.localizer.getPose().position.y/0.0254);
                    telemetry.addData("heading", Math.toDegrees(drive.localizer.getPose().heading.toDouble()));
                 //   beginPoseFound = true;
                }
                telemetry.addData("launchCycle Running", "");
                telemetry.addData("isValid", result.isValid());
                //telemetry.addData("unadjusted RPM", launcher.getVelocity(AngleUnit.DEGREES)/6);
                telemetry.addData("time", time.now(TimeUnit.SECONDS)-timer);
                telemetry.addData("startTime", startTime);
                telemetry.addData("toSpeed:", toSpeed);
                telemetry.addData("Is Launching:", launchStarted);
                telemetry.addData("TARGET LAUNCH RPM", power);
                //telemetry.addData("LAUNCH RPM", launcher.getVelocity(AngleUnit.DEGREES));
                telemetry.addData("LAUNCH RPM ADJ", launcher.getVelocity(AngleUnit.DEGREES) * 19.1);
                telemetry.addData("artifacts", artifacts);
                telemetry.addData("index", index);
                telemetry.addData("color", color1.green());
                telemetry.update();
                if (!auto) {
                    drive.setDrivePowers(new PoseVelocity2d(
                            new Vector2d(
                                    -gamepad2.left_stick_y,
                                    -gamepad2.left_stick_x
                            ),
                            -gamepad2.right_stick_x
                    ));
                    if (gamepad1.right_bumper) {
                        //   if
                        /* turret1.setPosition((gamepad1.left_stick_x+0.5));
                         */
                        //turret2.setPosition((gamepad1.left_stick_x+0.5));
                        //turret1.setPower(-gamepad1.left_stick_x);
                        //turret2.setPower(gamepad1.left_stick_x);
                        //turret1.setPower();
                        telemetry.addLine("Turret Running");
                    } else {
                        //turret1.setPower(0);
                        //turret2.setPower(0);
                    }
                    if (gamepad1.left_bumper) {
                        //hood.setPosition(clamp(gamepad1.left_stick_y, 0.25, 0.95));
                    }
                }
                //hood.setPosition(0.25);

                if (toSpeed && time.now(TimeUnit.SECONDS) - timer < startTime+1.9) {
                    if (auto) {
                        indexer.setPower(1);
                    }else {
                        indexer.setPower(0.5);
                    }
                }else if (time.now(TimeUnit.SECONDS)-timer >10 && time.now(TimeUnit.SECONDS)-timer <12){
                    if (auto) {
                        indexer.setPower(1);
                    }else {
                        indexer.setPower(0.5);
                    }
                }else if (time.now(TimeUnit.SECONDS)-timer > startTime+1.9 || time.now(TimeUnit.SECONDS)-timer >12) {
                    //indexer.setPower(0);

                    artifacts = Arrays.asList(0,0,0);
                    feedOff();
                    if(!auto){
                        index = PARAMS.indexCycleStart;
                        indexer(index);
                        setLaunchRPM(0);
                    }
                    break;
                }


            }
            return false;
        }
    }
    public class setPowers implements Action {
      //  @Override;

        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            ElapsedTime time = new ElapsedTime(ElapsedTime.Resolution.SECONDS);
            double timer = time.now(TimeUnit.SECONDS);
            while (time.now(TimeUnit.SECONDS) - timer < 20) {
                drive.setDrivePowers(new PoseVelocity2d(
                        new Vector2d(
                                -gamepad2.left_stick_y,
                                -gamepad2.left_stick_x
                        ),
                        -gamepad2.right_stick_x
                ));
            }
            return false;
        }
    }

}
