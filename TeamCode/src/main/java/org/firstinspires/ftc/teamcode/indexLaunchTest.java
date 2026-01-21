package org.firstinspires.ftc.teamcode;

import static androidx.core.math.MathUtils.clamp;
import static java.lang.Math.abs;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.teamcode.subsystems.MecanumDrive;

@TeleOp(name="indexLaunchTest")
public class indexLaunchTest extends robot {
    double kP = 0.0;
    double kI = 0.0;
    double kD= 0.0;

    // State variables
    double lastError = 0;
    double integralSum = 0;

    // A timer to calculate the change in time (delta time)
    ElapsedTime timer = new ElapsedTime();
    @Override
    public void runOpMode() throws InterruptedException {
        //DcMotorEx motor = hardwareMap.get(DcMotorEx.class, "motor");


        launcher.setVelocityPIDFCoefficients(0.3,0.0,0.3, 4);
        double Kp = 0.0045;
        double Ki = 0.0001028;
        //double Kd = 0.000000045;
        double Kd = 0.0000135;
        boolean x = false;
        int index = 0;
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

        /*
         * Starts polling for data.  If you neglect to call start(), getLatestResult() will return null.
         */
        limelight.start();
        LLResult result = limelight.getLatestResult();
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
        MecanumDrive drive = new MecanumDrive(hardwareMap, new Pose2d(new Vector2d(0,0), 0));
     //b   limelight.pipelineSwitch(0);
        waitForStart();
        while (opModeIsActive()) {
            result = limelight.getLatestResult();
            if (gamepad1.right_trigger > 0.0) {
                launcher.setVelocity(-gamepad1.right_stick_y * 6000, AngleUnit.DEGREES);
            }else {
                launcher.setPower(0.0);
            }
            if (gamepad1.left_trigger > 0) {
                launchFeedL.setPower(1);
                launchFeedR.setPower(1);
            }
            if (gamepad1.left_bumper){
                hood.setPosition(clamp(gamepad1.left_stick_y, 0.0, 0.95));
            }
            if (gamepad1.y) {
                intake.setPower(1);
            }else {
                intake.setPower(0);
            }

            if (gamepad1.x && !x){
               if (index <3) {
                   index += 1;
               } else {
                   index = 0;
               }
               indexer(index, true);
               x = true;
            } else if (!gamepad1.x) {
                x = false;
            }
            if (gamepad1.dpad_up) {
                Kp += 0.0001;
            } else if (gamepad1.dpad_down) {
                Kp -= 0.0001;
            }
            if (gamepad1.dpad_left) {
                Ki -= 0.0000001;
            } else if (gamepad1.dpad_right) {
                Ki += 0.0000001;
            }
            if (gamepad2.dpad_up){
                Kd += 0.0000001;
            } else if (gamepad2.dpad_down) {
                Kd -= 0.0000001;
            }

          //  double TX2 = 0;
            //double TX3 = 0;
            if (result != null && result.isValid()) {
                TX = result.getTx();
                telemetry.addData("Target X", TX);
               // telemetry.addData("Target X2", TX2);
                //telemetry.addData("Target X3", TX3);
                if (gamepad1.a) {
                    if (TX >1) {
                        turret2.setPower((TX/25)*0.5);
                    } else if (TX <-1) {
                        turret2.setPower((TX/25)*0.5);
                    } else {
                        turret2.setPower(0);
                    }
                } else {
                    turret2.setPower(0);
                }
                if (gamepad1.b) {
                    turret2.setPower(-pid.PIDControl(Kp,Ki, Kd, 0.0, TX));
                }
                turretYawRobot = result.getBotpose().getOrientation().getYaw(AngleUnit.DEGREES) - drive.localizer.getPose().heading.toDouble();
                turretYawRobot2 = result.getFiducialResults().get(0).getTargetXDegrees() - drive.localizer.getPose().heading.toDouble();
               // telemetry.addData("Target Y", ty);
                //telemetry.addData("Target Area", ta);
            } else {
                telemetry.addData("Limelight", "No Targets");
                //turret2.getController().getServoPosition(2);
                if (gamepad1.b) {
                    turret2.setPower(
                            -pid.PIDControl(Kp, Ki, Kd, Math.atan2(drive.localizer.getPose().position.x - (-58.31), drive.localizer.getPose().position.y - (55.64)), turretYawRobot + drive.localizer.getPose().heading.toDouble())
                    );
                }else {
                    turret2.setPower(0);
                }
            }
            if (abs(TX) < 4) {
                telemetry.addData("Target:", "Acquired");
            }
            if (gamepad1.right_bumper){
                //   if
                /* turret1.setPosition((gamepad1.left_stick_x+0.5));
                 */
                //turret2.setPosition((gamepad1.left_stick_x+0.5));
                turret1.setPower(gamepad1.left_stick_x);
                turret2.setPower(gamepad1.left_stick_x);
                //turret1.setPower();
                telemetry.addLine("Turret Running");
            } else {
                turret1.setPower(0);
                //  turret2.setPower(0);
            }
            drive.setDrivePowers(new PoseVelocity2d(
                    new Vector2d(
                            -gamepad2.left_stick_y,
                            -gamepad2.left_stick_x
                    ),
                    -gamepad2.right_stick_x
            ));
            telemetry.addData("calculated angle to red", Math.atan2(drive.localizer.getPose().position.x-(-58.31), drive.localizer.getPose().position.y - (55.64)));
            telemetry.addData("turretYawRobot", turretYawRobot);
            telemetry.addData("turretYawField", turretYawRobot+drive.localizer.getPose().heading.toDouble());
            telemetry.addData("turretYawField2", turretYawRobot2);
            telemetry.addData("Power", launcher.getPower());
            telemetry.addData("Launch Current", launcher.getCurrent(CurrentUnit.AMPS));
            telemetry.addData("RPM", launcher.getVelocity(AngleUnit.DEGREES));
            telemetry.addData("RPM Adjusted", launcher.getVelocity(AngleUnit.DEGREES)*20);
            telemetry.addData("TargetRPM", gamepad1.right_stick_y*6000);
            telemetry.addData("Right Stick Y", gamepad1.right_stick_y);
            //telemetry.addData("Left Trigger", gamepad1.left_trigger);
            telemetry.addData("Current", launcher.getCurrent(CurrentUnit.AMPS));
            telemetry.addData("Hood Position", hood.getPosition());
            telemetry.addData("turret1", turret1.getPower());
            telemetry.addData("turret2", turret2.getPower());
            telemetry.addData("index", indexer.getPosition());
            telemetry.addData("Kp", Kp);
            telemetry.addData("Ki x1000", Ki*1000);
            telemetry.addData("Kd x1000", Kd*1000);
           // telemetry.addData("turret1pow", turret1.getController().getPwmStatus());
            telemetry.update();

        }
    }
   
    // A timer to calculate the change in time (delta time)
   // ElapsedTime timer = new ElapsedTime();
    public double calculate(double target, double current) {
        // Calculate the error
        double error = target - current;

        // Calculate the derivative (rate of change of error)
        double derivative = (error - lastError) / timer.seconds();

        // Calculate the integral (sum of all past errors)
        // This helps correct for steady-state error.
        integralSum = integralSum + (error * timer.seconds());

        // The PID formula
        double output = (kP * error) + (kI * integralSum) + (kD * derivative);

        // Update state for the next loop
        lastError = error;
        timer.reset(); // Reset the timer for the next calculation

        return output;
    }
}
