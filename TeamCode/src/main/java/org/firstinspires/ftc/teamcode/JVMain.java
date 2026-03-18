package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

@TeleOp(name="!!! JV Main !!!")
public class JVMain extends OpMode {

    private DcMotor front_left, front_right, back_left, back_right;
    private DcMotorEx launch;
    private CRServo leftFeed, rightFeed;
    private DcMotorEx intake;

    // --- Motor Constants ---
    static final double TICKS_PER_REV = 537.7;

    // --- SETTABLE POWER VARIABLES ---
    // Changed from RPM to Power (0.0 to 1.0)
    private double activeTargetPower = 0;
    private boolean dpadUpPressed = false;
    private boolean dpadDownPressed = false;

    @Override
    public void init() {
        front_left   = hardwareMap.get(DcMotor.class, "fldrive");
        front_right  = hardwareMap.get(DcMotor.class, "frdrive");
        back_left    = hardwareMap.get(DcMotor.class, "bldrive");
        back_right   = hardwareMap.get(DcMotor.class, "brdrive");

        intake = hardwareMap.get(DcMotorEx.class, "intake");
        launch = hardwareMap.get(DcMotorEx.class, "launch");
        // BRAKE is critical for driving accuracy
        front_left.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        front_right.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        back_left.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        back_right.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // CHANGED: Use RUN_WITHOUT_ENCODER for direct setPower control
        // (This stops the internal PID from trying to correct velocity)
        launch.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        leftFeed = hardwareMap.get(CRServo.class, "leftFeed");
        rightFeed = hardwareMap.get(CRServo.class, "rightFeed");

        telemetry.addData("Status", "Initialized");

        front_left.setDirection(DcMotor.Direction.REVERSE);
        front_right.setDirection(DcMotor.Direction.REVERSE);
        // back_left.setDirection(DcMotor.Direction.REVERSE);
        back_right.setDirection(DcMotor.Direction.REVERSE);

        RevHubOrientationOnRobot orientationOnRobot = new
                RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.LEFT,
                RevHubOrientationOnRobot.UsbFacingDirection.UP
        );
    }



    @Override
    public void loop() {
        // 1. DRIVE INPUTS
        double drive  = -gamepad1.left_stick_y;
        double strafe = gamepad1.left_stick_x;
        double twist  = -gamepad1.right_stick_x;




        // 2. POWER SETTING LOGIC (D-Pad Up/Down to adjust)
        // We now increment by small decimals (e.g. 0.05 for 5% power)
        if (gamepad1.dpad_up && !dpadUpPressed) {
            activeTargetPower += 0.05;
            dpadUpPressed = true;
        } else if (!gamepad1.dpad_up) {
            dpadUpPressed = false;
        }

        if (gamepad1.dpad_down && !dpadDownPressed) {
            activeTargetPower -= 0.05;
            dpadDownPressed = true;
        } else if (!gamepad1.dpad_down) {
            dpadDownPressed = false;
        }

        // Clamp Power so it stays between 0.0 and 1.0
        if (activeTargetPower < 0) activeTargetPower = 0;
        if (activeTargetPower > 1.0) activeTargetPower = 1.0;

        // 3. LAUNCHER CONTROL
        // Trigger acts as a throttle for the set Power Limit
        double currentPower = gamepad1.right_trigger * activeTargetPower;

        // CHANGED: Use setPower directly.
        // We keep the negative sign because your original code used negative velocity.
        launch.setPower(-currentPower);

        //intake
        if(gamepad1.left_trigger>0.1) {
            intake.setPower(1);
        }else{intake.setPower(0);}

        // 4. MECANUM MATH
        double[] speeds = {
                (drive + strafe + twist),
                (drive - strafe - twist),
                (drive - strafe + twist),
                (drive + strafe - twist)
        };

        double max = 1.0;
        for (double s : speeds) max = Math.max(max, Math.abs(s));
        for (int i = 0; i < speeds.length; i++) speeds[i] /= max;

        front_left.setPower(speeds[0]);
        front_right.setPower(speeds[1]);
        back_left.setPower(speeds[2]);
        back_right.setPower(speeds[3]);

        // 5. INTAKE
        if (gamepad1.a) {
            leftFeed.setPower(1.0);
            rightFeed.setPower(1.0);
        }
        else if (gamepad1.y) {
            leftFeed.setPower(-1.0);
            rightFeed.setPower(-1.0);
        } else {
            leftFeed.setPower(0.0);
            rightFeed.setPower(0.0);
        }

        // 6. TELEMETRY
        telemetry.addData("Target Power Limit", "%.2f", activeTargetPower);
        telemetry.addData("Applied Power", "%.2f", currentPower);

        // We can still calculate actual RPM for monitoring purposes
        // (This only works if the encoder cable is still plugged in)
        double actualRPM = (launch.getVelocity() * 60.0) / TICKS_PER_REV;
        telemetry.addData("Actual RPM", "%.0f", Math.abs(actualRPM));

        telemetry.update();
    }
}