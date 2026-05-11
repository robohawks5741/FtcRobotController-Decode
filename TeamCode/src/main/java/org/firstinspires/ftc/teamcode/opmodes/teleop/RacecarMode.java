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

    public static double steeringMultiplier = 0.9;

    public static String speedIndicator(int length, double percent) {
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

    public static String steeringIndicator(int length, double percent) {
        if (length <= 0) return "";

        // Clamp to -100% through 100%
        percent = Math.max(-100, Math.min(100, percent));

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

        char[] bar = new char[length];
        for (int i = 0; i < length; i++) {
            bar[i] = ' ';
        }

        int middle = length / 2;

        if (percent > 0) {
            // Fill to the right from the middle
            int rightLength = length - middle;
            double filled = rightLength * (percent / 100.0);

            int fullBlocks = (int) filled;
            int partialIndex = (int) Math.round((filled - fullBlocks) * 8);

            if (partialIndex == 8) {
                fullBlocks++;
                partialIndex = 0;
            }

            for (int i = 0; i < fullBlocks && middle + i < length; i++) {
                bar[middle + i] = '█';
            }

            StringBuilder result = new StringBuilder(new String(bar));

            if (middle + fullBlocks < length && partialIndex > 0) {
                result.replace(
                        middle + fullBlocks,
                        middle + fullBlocks + 1,
                        partials[partialIndex]
                );
            }

            return result.toString();
        }

        if (percent < 0) {
            // Fill to the left from the middle
            int leftLength = middle;
            double filled = leftLength * (-percent / 100.0);

            int fullBlocks = (int) filled;
            int partialIndex = (int) Math.round((filled - fullBlocks) * 8);

            if (partialIndex == 8) {
                fullBlocks++;
                partialIndex = 0;
            }

            for (int i = 0; i < fullBlocks && middle - 1 - i >= 0; i++) {
                bar[middle - 1 - i] = '█';
            }

            StringBuilder result = new StringBuilder(new String(bar));

            if (middle - 1 - fullBlocks >= 0 && partialIndex > 0) {
                result.replace(
                        middle - 1 - fullBlocks,
                        middle - fullBlocks,
                        partials[partialIndex]
                );
            }

            return result.toString();
        }

        return new String(bar);
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
                case 1: // normal driving
                    telemetry.addData("Driving Mode", "Normal");
                    telemetry.addLine("Controls: Left stick Y for forward/back, right stick X for steering");
                    break;
                case 2: // racing mode
                    telemetry.addData("Driving Mode", "Racing");
                    telemetry.addLine("Controls: Right stick X for steering, left trigger for gas pedal, right trigger for brake pedal");
                    break;
                case 3: // electric mode
                    telemetry.addData("Driving Mode", "Electric");
                    telemetry.addLine("Controls: Right stick X for steering, left trigger for throttle, right trigger for brake/reverse");
                    break;
            }

            telemetry.addLine();

            double dt = timer.seconds();
            timer.reset();

            double throttle = gamepad1.left_trigger;  // accelerator
            double brake = gamepad1.right_trigger;    // brake
            double rawSteer = gamepad1.right_stick_x;
            double steer, speedRatio, steerLimit, servoPosition;

            switch (drivingMode) {
                case 1: // normal driving
                    backLeftMotor.setPower(gamepad1.left_stick_y);
                    backRightMotor.setPower(gamepad1.left_stick_y);
                    steeringServo.setPosition(gamepad1.right_stick_x * steeringMultiplier);

                    telemetry.addData("Speed", speedIndicator(10, Math.abs(gamepad1.left_stick_y) * 100));
                    telemetry.addData("Steering", steeringIndicator(20, (gamepad1.right_stick_x * 100)));
                    break;

                case 2: // racing mode
                    double gas = throttle;    // 0.0 to 1.0

                    gas = gas * gas;
                    brake = brake * brake;

                    if (gas > 0.02) {
                        RacingMode.speed += gas * RacingMode.accelRate * dt;
                    } else if (brake > 0.02) {
                        if (RacingMode.speed > 0) {
                            RacingMode.speed -= brake * RacingMode.brakeRate * dt;
                        } else {
                            // reverse after stopping
                            RacingMode.speed -= brake * RacingMode.accelRate * 0.6 * dt;
                        }
                    } else {
                        // coasting drag
                        if (RacingMode.speed > 0) {
                            RacingMode.speed -= (RacingMode.coastDrag * Math.abs(RacingMode.speed) + RacingMode.rollingDrag) * dt;
                            if (RacingMode.speed < 0) RacingMode.speed = 0;
                        } else if (RacingMode.speed < 0) {
                            RacingMode.speed += (RacingMode.coastDrag * Math.abs(RacingMode.speed) + RacingMode.rollingDrag) * dt;
                            if (RacingMode.speed > 0) RacingMode.speed = 0;
                        }
                    }

                    // clamp speed
                    RacingMode.speed = Math.max(RacingMode.maxReversePower, Math.min(RacingMode.maxForwardPower, RacingMode.speed));

                    // send simulated speed to drive motors
                    backLeftMotor.setPower(RacingMode.speed);
                    backRightMotor.setPower(RacingMode.speed);

                    telemetry.addData("Speed", speedIndicator(10, RacingMode.speed * 100));

                    rawSteer = gamepad1.right_stick_x;

                    // expo steering: smoother near center, still allows full steering
                    steer = Math.signum(rawSteer) * Math.pow(Math.abs(rawSteer), RacingMode.steeringExpo);

                    // reduce steering at higher speed
                    speedRatio = Math.abs(RacingMode.speed); // 0 to 1
                    steerLimit = RacingMode.maxSteer - (RacingMode.maxSteer - RacingMode.minSteer) * speedRatio;

                    steer *= steerLimit;

                    // convert -1..1 steering to servo position 0..1
                    double servoCenter = 0.5;
                    double servoRange = 0.35; // tune this so it doesn't oversteer mechanically

                    servoPosition = servoCenter + steer * servoRange;
                    servoPosition = Math.max(0.0, Math.min(1.0, servoPosition * steeringMultiplier));

                    steeringServo.setPosition(servoPosition);

                    telemetry.addData("Steering", steeringIndicator(20, (Math.min((servoPosition + 0.5), 1) * 100)));
                    break;
                case 3: // electric car mode
                    // optional EV throttle curve.
                    // EVs usually feel responsive, so don't square it too aggressively
                    throttle = Math.pow(throttle, 1.4);
                    brake = brake * brake;

                    if (throttle > 0.02) {
                        // instant-feeling EV torque
                        // acceleration tapers slightly as speed gets higher
                        double speedFactor = 1.0 - 0.35 * Math.abs(ElectricMode.speed);

                        if (ElectricMode.speed >= 0) {
                            ElectricMode.speed += throttle * ElectricMode.accelRate * speedFactor * dt;
                        } else {
                            // if rolling backward, throttle first slows you down
                            ElectricMode.speed += throttle * ElectricMode.brakeRate * dt;
                        }

                    } else if (brake > 0.02) {
                        if (ElectricMode.speed > ElectricMode.brakeToReverseDelay) {
                            // normal brake while moving forward
                            ElectricMode.speed -= brake * ElectricMode.brakeRate * dt;
                        } else {
                            // once nearly stopped, brake becomes reverse
                            ElectricMode.speed -= brake * ElectricMode.reverseAccelRate * dt;
                        }

                    } else {
                        // regenerative braking when neither pedal is pressed
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

                        // tiny rolling drag after regen
                        if (ElectricMode.speed > 0) {
                            ElectricMode.speed -= ElectricMode.rollingDrag * dt;
                            if (ElectricMode.speed < 0) ElectricMode.speed = 0;
                        } else if (ElectricMode.speed < 0) {
                            ElectricMode.speed += ElectricMode.rollingDrag * dt;
                            if (ElectricMode.speed > 0) ElectricMode.speed = 0;
                        }
                    }

                    // clamp speed
                    ElectricMode.speed = Math.max(ElectricMode.maxReversePower, Math.min(ElectricMode.maxForwardPower, ElectricMode.speed));

                    steer = Math.signum(rawSteer) * Math.pow(Math.abs(rawSteer), ElectricMode.steeringExpo);

                    // reduce steering authority at high speed
                    speedRatio = Math.abs(ElectricMode.speed);
                    steerLimit = ElectricMode.maxSteer - (ElectricMode.maxSteer - ElectricMode.minSteer) * speedRatio;
                    steer *= steerLimit;

                    servoPosition = ElectricMode.servoCenter + steer * ElectricMode.servoRange;
                    servoPosition = Math.max(0.0, Math.min(1.0, servoPosition * steeringMultiplier));

                    backLeftMotor.setPower(ElectricMode.speed);
                    backRightMotor.setPower(ElectricMode.speed);
                    steeringServo.setPosition(servoPosition);

                    telemetry.addData("Speed        ", speedIndicator(10, ElectricMode.speed * 100));
                    telemetry.addData("Steering", steeringIndicator(20, (Math.min((servoPosition + 0.5), 1) * 100)));
                    break;
            }

            telemetry.update();
        }
    }
}