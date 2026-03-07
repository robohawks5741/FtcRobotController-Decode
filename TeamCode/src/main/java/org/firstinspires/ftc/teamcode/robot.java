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

import android.sax.StartElementListener;

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
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.matrices.VectorF;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.subsystems.MecanumDrive;
import org.firstinspires.ftc.teamcode.subsystems.PinpointLocalizer;
import org.firstinspires.ftc.vision.apriltag.AprilTagGameDatabase;
import org.firstinspires.ftc.vision.apriltag.AprilTagLibrary;

import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import java.util.Arrays;
import java.util.List;
import java.util.concurrent.TimeUnit;

@TeleOp(name = "Robot", group = "Robot")
@Disabled
public class robot extends LinearOpMode {

    AprilTag aprilTag;
    AnalogInput indexFB;
    AnalogInput turretFB;
    DcMotorEx launcher;
    DcMotorEx liftL;
    DcMotorEx liftR;
    double turnVector;
    DcMotorEx intake;
    CRServo launchFeedL;
    CRServo launchFeedR;
    CRServo indexer;
    public CRServo turret1;
    //CRServo turret2;
    Servo hood;
    Servo feed;
    ColorSensor color;
    public MecanumDrive drive;

    public double TICKS_PER_REV = 384.5;

    // PIDF for launcher motors (example values, tune these)
    public static final PIDFCoefficients LAUNCH_PIDF =
            new PIDFCoefficients(10, 3, 0.5, 12);

    int lastPosLeft;
    int lastPosRight;
    long lastTime;
    int index = 0;
    int power = 3400;
    boolean isLaunching;
    PinpointLocalizer PinpointLocalizer;
    IMU imu;
    double lastIndexPosition;
    double totalIndexRotation = 0;
    int totalRotations = 0;
    double lastTurretPosition = 0;
    double totalTurretRotation = 0;
    int totalTurretRotations = 0;
    double lastTargetAngle = 0;
    public int modifier = 1;


    int rotationTicker = 0;
    int lastIndex = 0;
    boolean indexCC = true;
    public static class Params {
        

        public double beginPosX = 65;
        public double beginPosY = 11;
        public double localizerX = beginPosX;
        public double localizerY = beginPosY;
        public double beginHeading = Math.toRadians(180);
        public double targetX = -53;
        public double targetY = 25;
        public double row1X = 34.5;
        public double row1Y = 68;
        public double row2X = 10;
        public double row2Y = 69;
        public double row3X = -13;
        public double row3Y = 62;
        public double backToY = 25;
        public double row1CheckY = row1Y;
        public double row2CheckY = row2Y;
        public double row3CheckY = row3Y;
        public double intakeHeading = Math.toRadians(90);
        public double endX = 30;
        public double endY = -20;
        public double targetHeading = 110;
        public double Kp = 0.01;
        public double Ki = 0.0000001;
        //double Kd = 0.000000045;
        public double Kd = 0.4;
        public double indexerP = 0.000000001;
        public double indexerI = 0.0000000000000000001;
        public double indexerD = 0.00000003;
        public double turretP = 0.011;
        //turretP as of 3/6/2026: 0.006; (WORKING)
        public double turretI = 0.00000000000000000001;
        //turretI as of 3/6/2026: 0.00000000000000000001;(WORKING)
        public double turretD = 0.0000000;
        //turretD as of 3/6/2026: 0.0000000;(WORKING)
        public RevHubOrientationOnRobot orientationOnRobot;
        public int indexCycleStart = 0;





    }
    public boolean isRedAlliance = false;
    public static Params PARAMS = new Params();
    public double turretAngle = 0;
    public int teleopPower = 4000;
    public int autoPower = 2150;
    public double hoodPosition = 0;
    public Pose2d beginPos = new Pose2d(new Vector2d(PARAMS.beginPosX, PARAMS.beginPosY), PARAMS.beginHeading);
    public static Pose2d teleOpBeginPose;

    public Pose2d globalLoc() {
        double x = PinpointLocalizer.getPose().position.x;
        double y = PinpointLocalizer.getPose().position.y;
        double theta = PinpointLocalizer.getPose().heading.toDouble();
        return new Pose2d(x, y, theta);
    }
    PID pid;
    PID indexPID;
    PID turretPID;
    Limelight3A limelight;
    LLResult result;
    public AprilTagLibrary tagLibrary = AprilTagGameDatabase.getDecodeTagLibrary();

    public AprilTagLibrary getTagLibrary() {
        return tagLibrary;
    }
    VectorF redTagVector = getTagLibrary().lookupTag(24).fieldPosition;
    double redTagX = redTagVector.get(0);
    double redTagY = redTagVector.get(1);
    double redTagHeading = 0;
    double redTagDistance = 0;

    VectorF blueTagVector = getTagLibrary().lookupTag(20).fieldPosition;
    double blueTagX = blueTagVector.get(0);
    double blueTagY = blueTagVector.get(1);
    double blueTagHeading = 0;
    double blueTagDistance = 0;

    List<Integer> artifacts;
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
        color = hardwareMap.get(ColorSensor.class, "color");
        liftL = hardwareMap.get(DcMotorEx.class, "liftL");
        liftR = hardwareMap.get(DcMotorEx.class, "liftR");
        //  turret1.scaleRange(0.25, .75);
        //turret2.scaleRange(.25, 0.75);
       // indexer.scaleRange(0, 1);
        // turret1.getController().
        turret1.setDirection(CRServo.Direction.FORWARD);
        turret1.setPower(0);

       // turret2.setDirection(CRServo.Direction.REVERSE);
        PinpointLocalizer = new PinpointLocalizer(hardwareMap, 0.00072471557, new Pose2d(0, 0, 0));
        PinpointLocalizer.driver.resetPosAndIMU();
        pid = new PID();
        indexPID = new PID();
        turretPID = new PID();
        launcher.setVelocityPIDFCoefficients(22, 0.3, 0, 12);
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
        if (result.isValid() && result != null) {
           // beginPos = new Pose2d(new Vector2d(result.getBotpose().getPosition().x*39.3701, result.getBotpose().getPosition().y*39.3701), Math.toRadians(result.getBotpose().getOrientation().getYaw()));
        }
        telemetry.addData("beginPosition", beginPos);
        telemetry.addData("LLValid", result.isValid());
        telemetry.update();
        //aprilTag = new AprilTag("Webcam 1", hardwareMap);
        waitForStart();
        drive = new MecanumDrive(hardwareMap, beginPos);
        drive.localizer.update();
        blueTagHeading = Math.atan2(blueTagY-PARAMS.localizerY, blueTagX-PARAMS.localizerX);
        blueTagDistance = Math.hypot(blueTagX-PARAMS.localizerX, blueTagY-PARAMS.localizerY);
        redTagHeading = Math.atan2(redTagY-PARAMS.localizerY, redTagX-PARAMS.localizerX);
        redTagDistance = Math.hypot(redTagX-PARAMS.localizerX, redTagY-PARAMS.localizerY);
        // aprilTag = new AprilTag("Webcam 1", hardwareMap);

        if (isStopRequested()){
            indexer(0);
        }
    }
    public class sendAutoEndPose implements Action{
        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            drive.localizer.update();
            teleOpBeginPose = drive.localizer.getPose();
            return false;
        }
    }
    public void launcherAngleVelocity(){
        double tagDistance;
        if (isRedAlliance){
            tagDistance = redTagDistance;
        } else {
            tagDistance = blueTagDistance;
        }
        double vel = (Math.PI*2*power)/60;
        hoodPosition = (((0.7*tagDistance)/145)+0.25);
        double ang = Math.atan((Math.pow(vel, 2)+Math.sqrt(Math.pow(vel,4) - 9.81*(9.8*Math.pow(tagDistance, 2)+ (2*1*Math.pow(vel,2)))))/(9.8*tagDistance));
        if (ang < 30) {ang = 30;}
        if (ang > 80) {ang = 80;}
        //hoodPosition = 0.25 + (ang-30)*(0.95-0.25)/(80-30);
        teleopPower =  Math.toIntExact(Math.round((14.2*tagDistance)+1850));
        telemetry.addData("tagDistance", tagDistance);
        telemetry.addData("hoodPosition", hoodPosition);
        telemetry.addData("observedHoodPosition", hood.getPosition());
        telemetry.addData("teleopPower", teleopPower);
        hood.setPosition(clamp(hoodPosition, 0.25, .95));

    }
    public void setTurretPosition(double targetPosition) {
        double currentAngle = ((turretFB.getVoltage() / 3.3) * 360.0);
        double turretTargetAngle = targetPosition+8;
        double power;
        if (lastTurretPosition-currentAngle >=280){
            totalTurretRotations += 1;
        }else if(lastTurretPosition - currentAngle <= -280) {
            totalTurretRotations -=1;
        }
        totalTurretRotation = (currentAngle + totalTurretRotations*360)/4.167;

        power = -pid.turretPID(PARAMS.turretP, PARAMS.turretI, PARAMS.turretD, turretTargetAngle, totalTurretRotation);


        //turret1.setPower(clamp(power, -1, 1));
        turret1.setPower(power);
        lastTurretPosition = currentAngle;

        telemetry.addData("turretServoPower", turret1.getPower());
        telemetry.addData("Turret Target", turretTargetAngle);
        telemetry.addData("Turret Current", currentAngle);
      // telemetry.addData("Turret Current Adjusted", currentAngle)
        telemetry.addData("Turret Total", totalTurretRotation);
        telemetry.addData("Turret Target Adjusted", turretTargetAngle + 360*totalRotations);
        telemetry.addData("Turret Power", power);
        telemetry.addData("Turret Rotations", totalTurretRotations);


    }
    public void setIndexPosition(double targetPosition) {
       // indexer.setPosition(targetPosition);
      //  double targetPosition
        double currentAngle = (indexFB.getVoltage() / 3.3) * 360.0;
       // totalIndexRotation = currentAngle;
        double indexerTargetAngle = targetPosition*360;
       /* if (indexerTargetAngle == 0 && currentAngle > 260) {
            indexerTargetAngle = 360;
        }*/

        if (lastIndexPosition-currentAngle >=300){
            totalRotations += 1;
        } else if (lastIndexPosition - currentAngle <= -300) {
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
                case 5:
                    Actions.runBlocking(new SleepAction(15));
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

    public void indexer(int position) {
        if (position < lastIndex) {
            rotationTicker += 1;
        }
        switch (position){
            //number refers to which arm is where
            //eg. "1 at launch" means that arm 1 is at the launch and slot 1 is at the intake

            case 1:
                //3 at launch
                //setIndexPosition(0.167);
                pid.indexReset();
                setIndexPosition(0.21);

                break;
            case 2:
                //2 at intake
                pid.indexReset();
                setIndexPosition(0.375);
                break;

            case 3:
                pid.indexReset();
                setIndexPosition(0.60);
                break;
            case 4:
                //3 at intake
                pid.indexReset();
                setIndexPosition(0.74);
                break;
            case 5:
                //2 at launch
                //setIndexPosition(0.925);
                pid.indexReset();
                setIndexPosition(1.0);
                break;

            case 0:
                //1 at intake
                pid.indexReset();
                setIndexPosition(0);
                break;
            default:
                //1 at intake
                pid.indexReset();
                setIndexPosition(0.0);
        }
        lastIndex = position;
    }
    public class turretTrack implements Action {
       public boolean redAlliance;
        public turretTrack(boolean redAlliance){
            this.redAlliance = redAlliance;
        }
        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            if (redAlliance) {
                pid.turretReset();
                drive.localizer.update();
                turretAngle = Math.toDegrees(Math.atan2(Math.sin(Math.toRadians(redTagHeading)-drive.localizer.getPose().heading.toDouble()), Math.cos(Math.toRadians(redTagHeading)-drive.localizer.getPose().heading.toDouble())));
               // setTurretPosition(PARAMS.turretAngle);
                telemetry.addData("toRedGoal", turretAngle);
            } else {
                pid.turretReset();
                drive.localizer.update();
                turretAngle = Math.toDegrees(Math.atan2(Math.sin(Math.toRadians(blueTagHeading)-drive.localizer.getPose().heading.toDouble()), Math.cos(Math.toRadians(blueTagHeading)-drive.localizer.getPose().heading.toDouble())));

                // setTurretPosition(PARAMS.turretAngle);
                telemetry.addData("toBlueGoal", turretAngle);

            }
           // telemetry.addData("turretPower", power);
            return false;
        }
    }
    public void feedOn() {

        intake.setPower(1);


    }
    public void feedOff() {

        intake.setPower(0);
    }
    public void colorLogger() {
        int intakeSlotContents;
        int offset = -1;
        //0 = empty
        //1 = purple
        //2 = green
        double purple = (color.red()+color.blue())/2;
        double green = color.green();
        if (color.green() > 200 && green - purple > 100) {
            intakeSlotContents = 2;
        } else if (purple >200 && purple - green > 100) {
            intakeSlotContents = 1;
        } else {
            intakeSlotContents = 0;
        }
        isLaunching = abs(launcher.getVelocity()) > 10;
        if (isLaunching || intake.getPower() >0.1) {
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
        //telemetry.addData("artifacts", artifacts);
    }
    public class launchCycle implements Action {
        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            ElapsedTime time = new ElapsedTime(ElapsedTime.Resolution.SECONDS);
            double timer = time.now(TimeUnit.SECONDS);
            setLaunchRPM(power);
            index = 0;
            indexer(index);
            double longDelay;
            double shortDelay;
            feedOff();
            while (opModeIsActive()) {
                colorLogger();
                intake.setPower(1);
                telemetry.addData("launchCycle Running", "");
                //telemetry.addData("unadjusted RPM", launcher.getVelocity(AngleUnit.DEGREES)/6);
                telemetry.addData("LAUNCH RPM", launcher.getVelocity(AngleUnit.DEGREES));

                telemetry.update();
                drive.setDrivePowers(new PoseVelocity2d(
                        new Vector2d(
                                -gamepad2.left_stick_y,
                                -gamepad2.left_stick_x
                        ),
                        -gamepad2.right_stick_x
                ));
                if (gamepad1.right_bumper){
                    //   if
                    /* turret1.setPosition((gamepad1.left_stick_x+0.5));
                     */
                    //turret2.setPosition((gamepad1.left_stick_x+0.5));
                    turret1.setPower(-gamepad1.left_stick_x);
                    //turret2.setPower(gamepad1.left_stick_x);
                    //turret1.setPower();
                    telemetry.addLine("Turret Running");
                } else {
                    turret1.setPower(0);
                    //turret2.setPower(0);
                }
                if (gamepad1.left_bumper){
                    hood.setPosition(clamp(gamepad1.left_stick_y, 0.25, 0.95));
                }
                hood.setPosition(0.25);
                if (time.now(TimeUnit.SECONDS) - timer == 4) {
                    feedOn();
                }
                if (time.now(TimeUnit.SECONDS) - timer == 7) {
                    feedOff();
                    //indexer(2);
                }
                if (time.now(TimeUnit.SECONDS) - timer == 8) {

                    index = 2;
                    /*if (index >4) {
                        index =0;
                    }*/
                    indexer(index);
                }
                if (time.now(TimeUnit.SECONDS) - timer == 9) {
                    feedOn();
                }
                if (time.now(TimeUnit.SECONDS) - timer == 12) {
                    feedOff();
                    //indexer(4);
                }
                if (time.now(TimeUnit.SECONDS) - timer == 13) {
                    index = 4;
                   /* if (index >4 ) {
                        index =0;
                    }*/
                    indexer(index);
                }
                if (time.now(TimeUnit.SECONDS) - timer == 14) {
                    feedOn();
                }
                if (time.now(TimeUnit.SECONDS) - timer == 17) {
                    feedOff();


                }
                if (time.now(TimeUnit.SECONDS) - timer == 18) {

                    setLaunchRPM(0);
                    //indexer(3);
                    index = 3;
                    indexer(index);
                    // return true;

                }
                if (time.now(TimeUnit.SECONDS) - timer > 18) {
                    intake.setPower(0.0);
                    break;
                }

            }
            return false;
        }
    }
    public class newLaunchCycle implements  Action{
        public boolean auto;
        public newLaunchCycle(boolean auto) {
            this.auto = auto;
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
          //  indexer.setPower(0);
            if(auto){
                setLaunchRPM(power);
            }

            while (opModeIsActive()) {


                if (!auto) {
                    //new turretTrack(true).run(new TelemetryPacket());
                    launcherAngleVelocity();
                    power = teleopPower;
                    setLaunchRPM(power);
                    turretAngle -= gamepad1.right_stick_x*5;
                    setTurretPosition(turretAngle);
                    drive.localizer.update();
                    robot.PARAMS.localizerX = drive.localizer.getPose().position.x;
                    robot.PARAMS.localizerY = drive.localizer.getPose().position.y;
                    blueTagHeading = Math.toDegrees(Math.atan2(blueTagY-PARAMS.localizerY, blueTagX-PARAMS.localizerX));
                    blueTagDistance = Math.hypot(blueTagX-PARAMS.localizerX, blueTagY-PARAMS.localizerY);
                    redTagHeading = Math.toDegrees(Math.atan2(redTagY-PARAMS.localizerY, redTagX-PARAMS.localizerX));
                    redTagDistance = Math.hypot(redTagX-PARAMS.localizerX, redTagY-PARAMS.localizerY);



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
                telemetry.addData("color", color.green());
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

                if (toSpeed && time.now(TimeUnit.SECONDS) - timer < startTime+1.7) {
                   indexer.setPower(0.5);
                }else if (time.now(TimeUnit.SECONDS)-timer >10){
                    indexer.setPower(0.5);
                }else if (time.now(TimeUnit.SECONDS)-timer > startTime+1.7 || time.now(TimeUnit.SECONDS)-timer >12) {
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
