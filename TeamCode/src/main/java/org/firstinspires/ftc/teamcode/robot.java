package org.firstinspires.ftc.teamcode;

import android.util.Range;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.Rotation2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.MecanumDrive;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;

@TeleOp(name = "Robot", group = "Robot")
@Disabled
public class robot extends OpMode {

    AprilTag aprilTag;
    DcMotorEx launcherLeft;
    DcMotorEx launcherRight;

    DcMotorEx intake;
    CRServo launchFeed;

    MecanumDrive drive;

    public double TICKS_PER_REV = 384.5;

    // PIDF for launcher motors (example values, tune these)
    public static final PIDFCoefficients LAUNCH_PIDF =
            new PIDFCoefficients(10, 3, 0.5, 12);

    int lastPosLeft;
    int lastPosRight;
    long lastTime;

    PinpointLocalizer PinpointLocalizer;
    IMU imu;

    @Override
    public void init() {


        intake = hardwareMap.get(DcMotorEx.class, "intake");

        drive = new MecanumDrive(hardwareMap, new Pose2d(0, 0, 0));

        launcherLeft = hardwareMap.get(DcMotorEx.class, "launch1");
        launcherRight = hardwareMap.get(DcMotorEx.class, "launch2");
        launchFeed = hardwareMap.get(CRServo.class, "launchFeed");

        launcherLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        launcherRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // 🔄 Reverse left motor so its encoder counts correctly

        // Enable FTC's built-in PIDF control
        launcherLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        launcherRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        launcherLeft.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, LAUNCH_PIDF);
        launcherRight.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, LAUNCH_PIDF);

        PinpointLocalizer = new PinpointLocalizer(hardwareMap, 0.00072471557, new Pose2d(0, 0, 0));
        PinpointLocalizer.driver.resetPosAndIMU();

        launchFeed.setDirection(DcMotorSimple.Direction.FORWARD);

        imu = hardwareMap.get(IMU.class, "imu");

        RevHubOrientationOnRobot orientationOnRobot = new
                RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.UP,
                RevHubOrientationOnRobot.UsbFacingDirection.FORWARD
        );

        imu.initialize(new IMU.Parameters(orientationOnRobot));
        lastPosLeft = launcherLeft.getCurrentPosition();
        lastPosRight = launcherRight.getCurrentPosition();
        lastTime = System.nanoTime();
        // TODO: Make the start values based on the april tags

        double startX = 0;
        double startY = 0;
        double startTheta = 0;
        PinpointLocalizer.setPose(new Pose2d(startX, startY, startTheta));
        aprilTag = new AprilTag("Webcam 1", hardwareMap);


       // aprilTag = new AprilTag("Webcam 1", hardwareMap);

    }

    public Pose2d globalLoc() {
        double x = PinpointLocalizer.getPose().position.x;
        double y = PinpointLocalizer.getPose().position.y;
        double theta = PinpointLocalizer.getPose().heading.toDouble();
        return new Pose2d(x, y, theta);
    }

    @Override
    public void loop() {

        telemetry.addLine("Press A to reset Yaw");
        telemetry.addLine("Hold left bumper to drive in robot relative");
        telemetry.addLine("Left stick = translation");
        telemetry.addLine("Right stick = rotation");

        double forwardFactor = -gamepad1.left_stick_y;
        double rightFactor = gamepad1.left_stick_x;
        double turnFactor = gamepad1.right_stick_x;

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
    }

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

    public void setLaunchPower(double leftPower, double rightPower){
        launcherLeft.setPower(leftPower);
        launcherRight.setPower(rightPower);

    }

    // --- RPM Calculations for telemetry ---
    public double getRPMLeft() {
        long now = System.nanoTime();
        double dt = (now - lastTime) / 1e9;

        int currentPos = launcherLeft.getCurrentPosition();
        int deltaTicks = currentPos - lastPosLeft;

        lastPosLeft = currentPos;
        lastTime = now;

        return (deltaTicks / TICKS_PER_REV) / dt * 60.0;
    }

    public double getRPMRight() {
        long now = System.nanoTime();
        double dt = (now - lastTime) / 1e9;

        int currentPos = launcherRight.getCurrentPosition();
        int deltaTicks = currentPos - lastPosRight;

        lastPosRight = currentPos;
        lastTime = now;

        return (deltaTicks / TICKS_PER_REV) / dt * 60.0;
    }

    // --- BUILT-IN PIDF VELOCITY CONTROL ---
    public void setLaunchRPM(double leftRPM, double rightRPM) {
        launcherLeft.setVelocity(rpmToTicksPerSec(leftRPM));
        launcherRight.setVelocity(rpmToTicksPerSec(rightRPM));
    }

    public double rpmToTicksPerSec(double rpm) {
        return (rpm * TICKS_PER_REV) / 60.0;
    }
}
