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
import com.acmerobotics.roadrunner.Vector2d;
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
import org.firstinspires.ftc.teamcode.subsystems.MecanumDrive;
import org.firstinspires.ftc.teamcode.subsystems.PinpointLocalizer;

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
    DcMotorEx launcher;
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
    int power = 5000;
    boolean isLaunching;
    PinpointLocalizer PinpointLocalizer;
    IMU imu;
    double lastIndexPosition;
    double totalIndexRotation = 0;
    int totalRotations = 0;
    int rotationTicker = 0;
    boolean indexCC = true;
    public static class Params {
        public double beginPosX = 62;
        public double beginPosY = -22.5;
        public double Kp = 0.01;
        public double Ki = 0.0000001;
        //double Kd = 0.000000045;
        public double Kd = 0.4;
        public double indexerP = 0.00000025;
        public double indexerI = 0.00;
        public double indexerD = 0.0000005;
        public int indexCycleStart = 0;
    }
    public static robot.Params PARAMS = new robot.Params();



    public Pose2d globalLoc() {
        double x = PinpointLocalizer.getPose().position.x;
        double y = PinpointLocalizer.getPose().position.y;
        double theta = PinpointLocalizer.getPose().heading.toDouble();
        return new Pose2d(x, y, theta);
    }
    PID pid;
    PID indexPID;
    Limelight3A limelight;
    LLResult result;

    List<Integer> artifacts;
    @Override
    public void runOpMode() throws InterruptedException {
        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        result = limelight.getLatestResult();
        //pid = new PID();
        hood = hardwareMap.get(Servo.class, "hood");
        indexFB = hardwareMap.get(AnalogInput.class, "indexFB");
        // Servo feed = hardwareMap.get(Servo.class, "feed");
        intake = hardwareMap.get(DcMotorEx.class, "intake");
        turret1 = hardwareMap.get(CRServo.class, "turret1");
        //turret2 = hardwareMap.get(CRServo.class, "turret2");
        feed = hardwareMap.get(Servo.class, "feed");
        launchFeedL = hardwareMap.get(CRServo.class, "launchFeedL");
        launchFeedR = hardwareMap.get(CRServo.class, "launchFeedR");
        launcher = hardwareMap.get(DcMotorEx.class, "launcher");
        indexer = hardwareMap.get(CRServo.class, "index");
        color = hardwareMap.get(ColorSensor.class, "color");
        //  turret1.scaleRange(0.25, .75);
        //turret2.scaleRange(.25, 0.75);
       // indexer.scaleRange(0, 1);
        turret1.setDirection(CRServo.Direction.REVERSE);
       // turret2.setDirection(CRServo.Direction.REVERSE);
        PinpointLocalizer = new PinpointLocalizer(hardwareMap, 0.00072471557, new Pose2d(0, 0, 0));
        PinpointLocalizer.driver.resetPosAndIMU();
        pid = new PID();
        indexPID = new PID();
        //launcher.setVelocityPIDFCoefficients(10, 0.5, 0, 5);
      //  launchFeed.setDirection(DcMotorSimple.Direction.FORWARD);

        imu = hardwareMap.get(IMU.class, "imu");
        lastIndexPosition = (indexFB.getVoltage() / 3.3) * 360.0;
        artifacts = Arrays.asList(0, 0, 0);
        //0 = empty
        //1 = purple
        //2 = green


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
        PinpointLocalizer.setPose(new Pose2d(startX, startY, startTheta));
        imu.initialize(new IMU.Parameters(orientationOnRobot));
        //aprilTag = new AprilTag("Webcam 1", hardwareMap);
        waitForStart();
        drive = new MecanumDrive(hardwareMap, new Pose2d(0, 0, 0));
        // aprilTag = new AprilTag("Webcam 1", hardwareMap);


        /*while (opModeIsActive()) {
            PinpointLocalizer.update();
            telemetry.addLine("Press A to reset Yaw");
            telemetry.addLine("Hold left bumper to drive in robot relative");
            telemetry.addLine("Left stick = translation");
            telemetry.addLine("Right stick = rotation");

            double forwardFactor = -gamepad1.left_stick_y;
            double rightFactor = gamepad1.left_stick_x;
            double turnFactor = gamepad1.right_stick_x;
            List<AprilTagDetection> currentDetections = aprilTag.getDetectedTags();
            AprilTagDetection blueTag = null;
            AprilTagDetection redTag = null;

            for (AprilTagDetection detection: currentDetections) {
                if (detection.id == 20) {
                    blueTag = detection;
                } else if (detection.id == 24) {
                    redTag = detection;
                }
            }
            // Example shooter control (press X to run 3000 RPM)
            if (gamepad1.x) {
                setLaunchRPM(3000, 3000);
            } else {
                launcherLeft.setPower(0);
                launcherRight.setPower(0);
            }

            telemetry.addData("Left RPM", getRPMLeft());
            telemetry.addData("Right RPM", getRPMRight());
            telemetry.update();
        } */
        if (isStopRequested()){
            indexer(0);
        }
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
        power = -pid.indexPID(PARAMS.indexerP, PARAMS.indexerI, PARAMS.indexerD, indexerTargetAngle, totalIndexRotation);

        // Apply a deadband to prevent the servo from whining when it's at the target
        if (Math.abs(indexerTargetAngle - currentAngle) < 2.0) { // 2-degree tolerance
            power = 0;
        }

        // Set the CR Servo power
        indexer.setPower(clamp(power, -0.5, 0.5));

        lastIndexPosition = currentAngle;
        telemetry.addData("Indexer Target", indexerTargetAngle);
        telemetry.addData("Indexer Current", currentAngle);
        telemetry.addData("Indexer Total", totalIndexRotation);
        telemetry.addData("Indexer Target Adjusted", indexerTargetAngle + 360*totalRotations);
        telemetry.addData("Indexer Power", power);
        telemetry.addData("Indexer Rotations", totalRotations);


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


    // --- BUILT-IN PIDF VELOCITY CONTROL ---
    public void setLaunchRPM(double launchRPM) {
        launcher.setVelocity(-launchRPM/19.1, AngleUnit.DEGREES);
        if (launchRPM >0) {
            isLaunching = true;
        } else {
            isLaunching = false;
        }

    }

    public double rpmToTicksPerSec(double rpm) {
        return (rpm * TICKS_PER_REV) / 60.0;
    }

    public void indexer(int position) {
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
    }
    public class turretTrack implements Action {
        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            while (opModeIsActive()) {
              //  turret2.setPower(-pid.PIDControl(PARAMS.Kp, PARAMS.Ki, PARAMS.Kd, 0.0, limelight.getLatestResult().getTx()));
                turret1.setPower(pid.PIDControl(PARAMS.Kp, PARAMS.Ki, PARAMS.Kd, 0.0, limelight.getLatestResult().getTx()));
                if (abs(limelight.getLatestResult().getTx()) < 2) {
                    break;
                }
            }
            return false;
        }
    }
    public void feedOn() {
        feed.setPosition(0.2);
      //  launchFeedL.setPower(-1);
        //launchFeedR.setPower(1);
        intake.setPower(1);


    }
    public void feedOff() {
        feed.setPosition(0.3);
        //launchFeedL.setPower(0);
        //launchFeedR.setPower(0);
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
    public class newLaunchCycle implements  Action {
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
                telemetry.addData("LAUNCH RPM ADJ", launcher.getVelocity(AngleUnit.DEGREES) * 19.1);
                telemetry.addData("artifacts", artifacts);
                telemetry.addData("index", index);
                telemetry.addData("color", color.green());
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

                if (time.now(TimeUnit.SECONDS) - timer == 1) {
                    feedOn();
                    indexer.setPower(0);
                }
                if (time.now(TimeUnit.SECONDS) - timer == 4) {
                   indexer.setPower(1);
                }

                if (time.now(TimeUnit.SECONDS) - timer == 6) {
                    //indexer.setPower(0);
                    index = PARAMS.indexCycleStart;
                    indexer(index);
                    feedOff();
                    break;
                }


            }
            setLaunchRPM(0);
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
