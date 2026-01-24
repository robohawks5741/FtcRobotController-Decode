package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;

import java.util.List;


@TeleOp(name="#New MAIN")
public class newMain extends robot {
    @Override
    public void runOpMode() throws InterruptedException {
        super.runOpMode();
        while (opModeIsActive()) {
            PinpointLocalizer.update();
            List<AprilTagDetection> currentDetections = aprilTag.getDetectedTags();
            PID pid = new PID();
            double turnValue = -gamepad1.right_stick_x;


            Double launchPow;
        /*    if (gamepad1.right_trigger >= 0.1) {
                setLaunchPower(-1.0, 1.0);
            } else {
                setLaunchPower(0, 0);
            }*/

            if (gamepad1.left_trigger >= 0.1) {
                setIntakePower(-1);
            } else if (!(gamepad1.right_trigger > 0.1)) {
                setIntakePower(0);
            }

            if (gamepad1.right_trigger > 0.1) {
                launcher.setVelocity(-500);
                //launcherRight.setVelocity(-500);
                launchFeedL.setPower(-1);
                setIntakePower(-1);

            } else if (gamepad1.y) {
                launcher.setVelocity(-400);
                //launcherRight.setVelocity(-400);
            } else if (gamepad1.a) {
                launcher.setVelocity(-200);
                //launcherRight.setVelocity(-200);
            } else if (gamepad1.x) {
                launcher.setVelocity(-2000);
              //  launcherRight.setVelocity(-2000);
            } else if (gamepad1.dpad_left) {
                launcher.setVelocity(-1000);
            } else if (gamepad1.dpad_right) {
                launcher.setVelocity(0);
              //  launcherRight.setVelocity(-1000);
            } else {
                launcher.setVelocity(0);
                //launcherRight.setVelocity(0);
            }

            if (gamepad1.left_bumper) {
                launchFeedL.setPower(1);
            } else if (gamepad1.right_bumper) {
                launchFeedL.setPower(-1);
            } else if (!(gamepad1.right_trigger > 0.1)) {
                launchFeedL.setPower(0);
            }
            if (gamepad1.dpad_left) { // Aidan AprilTag code
                for (AprilTagDetection detection : currentDetections) {
                    if (detection.id == 20 || detection.id == 24) {
                        telemetry.addLine("GOAL visible");
                        turnVector = pid.PIDControl(0.5, 0.1, 0.1, 400, detection.center.x);
                        turnValue += turnVector;
                        telemetry.addData("Turn vector from PID: ", turnVector);
                    }
                }
            }
            if (gamepad1.dpad_right) { // Aidan AprilTag code
                for (AprilTagDetection detection : currentDetections) {
                    if (detection.id == 20 || detection.id == 24) {
                        telemetry.addLine("GOAL visible");
                        turnVector = pid.PIDControl(0.5, 0.1, 0.1, 400, detection.robotPose.getOrientation().getYaw(AngleUnit.DEGREES));
                        turnValue += turnVector;
                        telemetry.addData("Turn vector from PID: ", turnVector);
                    }
                }
            }
            drive.setDrivePowers(new PoseVelocity2d(
                    new Vector2d(
                            -gamepad1.left_stick_y,
                            -gamepad1.left_stick_x
                    ),
                    -turnValue
            ));
            telemetry.addData("globalLoc x", globalLoc().position.x);
            telemetry.addData("globalLoc y", globalLoc().position.y);
            telemetry.addData("globalLoc theta", Math.toDegrees(globalLoc().heading.toDouble()));
            telemetry.addData("pinpoint x", PinpointLocalizer.driver.getPosX(DistanceUnit.MM));
           // telemetry.addData("RPM Left clay", getRPMLeft());
            //telemetry.addData("RPM Right clay", getRPMRight());
            telemetry.addData("RPM Left", launcher.getVelocity(AngleUnit.DEGREES) * 60 / 360);
            //telemetry.addData("RPM Right", launcherRight.getVelocity(AngleUnit.DEGREES) * 60 / 360);
            telemetry.addData("Left Current", launcher.getCurrent(CurrentUnit.AMPS));
           // telemetry.addData("Right Current", launcherRight.getCurrent(CurrentUnit.AMPS));
          //  telemetry.addData("RPM TPS Left", rpmToTicksPerSec(getRPMLeft()));
            //telemetry.addData("RPM TPS Right", rpmToTicksPerSec(getRPMRight()));
            telemetry.addData("pos Left", launcher.getCurrentPosition());
            //telemetry.addData("pos Right", launcherRight.getCurrentPosition());
            telemetry.update();
            updateTelemetry(telemetry);
        }
    }
}
