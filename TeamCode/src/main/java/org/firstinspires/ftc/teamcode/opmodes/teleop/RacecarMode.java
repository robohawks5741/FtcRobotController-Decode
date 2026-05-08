package org.firstinspires.ftc.teamcode.opmodes.teleop;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp(name="Racecar Mode")
public class RacecarMode extends LinearOpMode {
    protected DcMotorEx backLeftMotor;
    protected DcMotorEx backRightMotor;

    protected Servo steeringServo;

    ElapsedTime timer = new ElapsedTime();

    protected int drivingMode = 1;
    protected ElapsedTime modeSwitchTimer = new ElapsedTime();
    protected double modeSwitchCooldown = 1.0;
    /*  driving modes:
          1. normal - left stick y directly corresponds to motor power, right stick x directly steers wheels instantly
          2. racing - left trigger is gas pedal, right trigger is brake, right stick x is steering
          3. electric - same controls as racing
     */

    public static class RacingMode {
        // Tunable values
        public static double maxForwardPower = 1.0;
        public static double maxReversePower = -0.45;

        public static double accelRate = 1.8;      // higher = faster acceleration
        public static double brakeRate = 3.5;      // higher = stronger braking
        public static double coastDrag = 0.65;     // natural slowdown when no gas/brake
        public static double rollingDrag = 0.08;   // small constant drag

        public static double steeringExpo = 1.8;   // higher = smoother near center
        public static double minSteer = 0.25;      // steering available at high speed
        public static double maxSteer = 1.0;       // steering available at low speed

        public static double speed = 0.0;          // simulated car speed, -1.0 to 1.0
    }

    public static class ElectricMode {
        public static double maxForwardPower = 1.0;
        public static double maxReversePower = -0.45;

        // EV-style acceleration
        public static double accelRate = 3.0;          // EVs accelerate quickly
        public static double reverseAccelRate = 1.3;

        // Regenerative braking
        public static double regenStrength = 1.4;      // slowdown when letting off throttle
        public static double maxRegenAtSpeed = 0.9;    // regen is stronger at speed
        public static double regenDeadband = 0.04;     // no regen if trigger is barely touched

        // Physical braking
        public static double brakeRate = 4.0;          // stronger than regen
        public static double brakeToReverseDelay = 0.08;

        // Natural drag
        public static double rollingDrag = 0.04;

        // Steering
        public static double steeringExpo = 1.7;
        public static double minSteer = 0.28;
        public static double maxSteer = 1.0;

        public static double servoCenter = 0.5;
        public static double servoRange = 0.35;

        public static double speed = 0.0;
    }

    public static String progressBar(int length, double percent) {
        if (length <= 0) return "";

        percent = Math.max(0, Math.min(100, percent));

        String[] partials = {
                "",   // 0/8
                "▏", // 1/8
                "▎", // 2/8
                "▍", // 3/8
                "▌", // 4/8
                "▋", // 5/8
                "▊", // 6/8
                "▉"  // 7/8
        };

        double filled = length * (percent / 100.0);

        int fullBlocks = (int) filled;
        int partialIndex = (int) Math.round((filled - fullBlocks) * 8);

        if (partialIndex == 8) {
            fullBlocks++;
            partialIndex = 0;
        }

        StringBuilder bar = new StringBuilder();

        for (int i = 0; i < fullBlocks && i < length; i++) {
            bar.append('█');
        }

        if (fullBlocks < length && partialIndex > 0) {
            bar.append(partials[partialIndex]);
        }

        while (bar.length() < length) {
            bar.append(' ');
        }

        return bar.toString();
    }

    @Override
    public void runOpMode() {
        backLeftMotor = hardwareMap.get(DcMotorEx.class, "backLeft");
        backRightMotor = hardwareMap.get(DcMotorEx.class, "backRight");
        steeringServo = hardwareMap.get(Servo.class, "steeringServo");

        backLeftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backRightMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        telemetry.addLine("Racecar Mode, not for use on primary robot");
        telemetry.addLine("Press A to switch driving mode");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {
            if (gamepad1.a && modeSwitchTimer.seconds() >= modeSwitchCooldown) {
                drivingMode = (drivingMode + 1) % 3;
                modeSwitchTimer.reset();
            }

            switch (drivingMode) {
                case 1:
                    telemetry.addData("Driving Mode", "Normal");
                    telemetry.addLine("Controls: Left stick Y for forward/back, right stick X for steering");
                    break;
            }

            double dt = timer.seconds();
            timer.reset();

            double throttle = gamepad1.left_trigger;  // accelerator
            double brake = gamepad1.right_trigger;    // brake
            double rawSteer = gamepad1.right_stick_x;

            switch (drivingMode) {
                case 1: // normal driving
                    backLeftMotor.setPower(gamepad1.left_stick_y);
                    backRightMotor.setPower(gamepad1.left_stick_y);
                    steeringServo.setPosition(gamepad1.right_stick_x);
                    break;

                case 2: // racing mode
                    double gas = throttle;    // 0.0 to 1.0

                    // Optional: make throttle less twitchy near the start
                    gas = gas * gas;
                    brake = brake * brake;

                    // Acceleration / braking model
                    if (gas > 0.02) {
                        RacingMode.speed += gas * RacingMode.accelRate * dt;
                    } else if (brake > 0.02) {
                        if (RacingMode.speed > 0) {
                            RacingMode.speed -= brake * RacingMode.brakeRate * dt;
                        } else {
                            // Reverse after stopping
                            RacingMode.speed -= brake * RacingMode.accelRate * 0.6 * dt;
                        }
                    } else {
                        // Coasting drag
                        if (RacingMode.speed > 0) {
                            RacingMode.speed -= (RacingMode.coastDrag * Math.abs(RacingMode.speed) + RacingMode.rollingDrag) * dt;
                            if (RacingMode.speed < 0) RacingMode.speed = 0;
                        } else if (RacingMode.speed < 0) {
                            RacingMode.speed += (RacingMode.coastDrag * Math.abs(RacingMode.speed) + RacingMode.rollingDrag) * dt;
                            if (RacingMode.speed > 0) RacingMode.speed = 0;
                        }
                    }

                    // Clamp speed
                    RacingMode.speed = Math.max(RacingMode.maxReversePower, Math.min(RacingMode.maxForwardPower, RacingMode.speed));

                    // Send simulated speed to drive motors
                    backLeftMotor.setPower(RacingMode.speed);
                    backRightMotor.setPower(RacingMode.speed);

                    rawSteer = gamepad1.right_stick_x;

                    // Expo steering: smoother near center, still allows full steering
                    double steer = Math.signum(rawSteer) * Math.pow(Math.abs(rawSteer), RacingMode.steeringExpo);

                    // Reduce steering at higher speed
                    double speedRatio = Math.abs(RacingMode.speed); // 0 to 1
                    double steerLimit = RacingMode.maxSteer - (RacingMode.maxSteer - RacingMode.minSteer) * speedRatio;

                    steer *= steerLimit;

                    // Convert -1..1 steering to servo position 0..1
                    double servoCenter = 0.5;
                    double servoRange = 0.35; // tune this so it doesn't oversteer mechanically

                    double servoPosition = servoCenter + steer * servoRange;
                    servoPosition = Math.max(0.0, Math.min(1.0, servoPosition));

                    steeringServo.setPosition(servoPosition);
                    break;
                case 3: // electric car mode


                    // Optional EV throttle curve.
                    // EVs usually feel responsive, so don't square it too aggressively.
                    throttle = Math.pow(throttle, 1.4);
                    brake = brake * brake;

                    // -----------------------------
                    // EV SPEED / TORQUE SIMULATION
                    // -----------------------------

                    if (throttle > 0.02) {
                        // Instant-feeling EV torque.
                        // Acceleration tapers slightly as speed gets higher.
                        double speedFactor = 1.0 - 0.35 * Math.abs(ElectricMode.speed);

                        if (ElectricMode.speed >= 0) {
                            ElectricMode.speed += throttle * ElectricMode.accelRate * speedFactor * dt;
                        } else {
                            // If rolling backward, throttle first slows you down.
                            ElectricMode.speed += throttle * ElectricMode.brakeRate * dt;
                        }

                    } else if (brake > 0.02) {
                        if (ElectricMode.speed > ElectricMode.brakeToReverseDelay) {
                            // Normal brake while moving forward
                            ElectricMode.speed -= brake * ElectricMode.brakeRate * dt;
                        } else {
                            // Once nearly stopped, brake becomes reverse
                            ElectricMode.speed -= brake * ElectricMode.reverseAccelRate * dt;
                        }

                    } else {
                        // Regenerative braking when neither pedal is pressed.
                        if (Math.abs(ElectricMode.speed) > ElectricMode.regenDeadband) {
                            double regenAmount = ElectricMode.regenStrength * Math.min(Math.abs(ElectricMode.speed), ElectricMode.maxRegenAtSpeed);

                            if (ElectricMode.speed > 0) {
                                ElectricMode.speed -= regenAmount * dt;
                                if (ElectricMode.speed < 0) ElectricMode.speed = 0;
                            } else if (ElectricMode.speed < 0) {
                                ElectricMode.speed += regenAmount * dt;
                                if (ElectricMode.speed > 0) ElectricMode.speed = 0;
                            }
                        }

                        // Tiny rolling drag after regen
                        if (ElectricMode.speed > 0) {
                            ElectricMode.speed -= ElectricMode.rollingDrag * dt;
                            if (ElectricMode.speed < 0) ElectricMode.speed = 0;
                        } else if (ElectricMode.speed < 0) {
                            ElectricMode.speed += ElectricMode.rollingDrag * dt;
                            if (ElectricMode.speed > 0) ElectricMode.speed = 0;
                        }
                    }

                    // Clamp speed
                    ElectricMode.speed = Math.max(ElectricMode.maxReversePower, Math.min(ElectricMode.maxForwardPower, ElectricMode.speed));

                    // -----------------------------
                    // STEERING SIMULATION
                    // -----------------------------

                    steer = Math.signum(rawSteer) * Math.pow(Math.abs(rawSteer), ElectricMode.steeringExpo);

                    // Reduce steering authority at high speed
                    speedRatio = Math.abs(ElectricMode.speed);
                    steerLimit = ElectricMode.maxSteer - (ElectricMode.maxSteer - ElectricMode.minSteer) * speedRatio;
                    steer *= steerLimit;

                    servoPosition = ElectricMode.servoCenter + steer * ElectricMode.servoRange;
                    servoPosition = Math.max(0.0, Math.min(1.0, servoPosition));

                    // -----------------------------
                    // OUTPUT
                    // -----------------------------

                    backLeftMotor.setPower(ElectricMode.speed);
                    backRightMotor.setPower(ElectricMode.speed);
                    steeringServo.setPosition(servoPosition);
            }

            //telemetry.addData("Throttle", progressBar(10, Math.abs(gamepad1.left_stick_y) * 100));
        }
    }
}