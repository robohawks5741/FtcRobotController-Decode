package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;

import java.util.List;


@TeleOp(name="#Clay MAIN")
public class claymain extends robot {
    public void loop() {
        PinpointLocalizer.update();

        drive.setDrivePowers(new PoseVelocity2d(
                new Vector2d(
                        -gamepad1.left_stick_y,
                        -gamepad1.left_stick_x
                ),
                -gamepad1.right_stick_x
        ));

        Double launchPow;
    /*    if (gamepad1.right_trigger >= 0.1) {
            setLaunchPower(-1.0, 1.0);
        } else {
            setLaunchPower(0, 0);
        }*/

        if (gamepad1.left_trigger >= 0.1){
            setIntakePower(-1);
        } else if (!(gamepad1.right_trigger > 0.1)){
            setIntakePower(0);
        }

        if (gamepad1.right_trigger > 0.1){
            launcherLeft.setVelocity(-500);
            launcherRight.setVelocity(-500);
            launchFeed.setPower(-1);
            setIntakePower(-1);

        } else if (gamepad1.y){
            launcherLeft.setVelocity(-400);
            launcherRight.setVelocity(-400);
        }
        else if (gamepad1.a){
            launcherLeft.setVelocity(-200);
            launcherRight.setVelocity(-200);
        }
        else if (gamepad1.x){
            launcherLeft.setVelocity(-2000);
            launcherRight.setVelocity(-2000);
        }

        else{
            launcherLeft.setVelocity(0);
            launcherRight.setVelocity(0);
        }

        if (gamepad1.left_bumper) {
            launchFeed.setPower(1);
        } else if (gamepad1.right_bumper){
            launchFeed.setPower(-1);
        } else if (!(gamepad1.right_trigger > 0.1)){
            launchFeed.setPower(0);
        }


        telemetry.addData("globalLoc x", globalLoc().position.x);
        telemetry.addData("globalLoc y", globalLoc().position.y);
        telemetry.addData("globalLoc theta", Math.toDegrees(globalLoc().heading.toDouble()));
        telemetry.addData("pinpoint x", PinpointLocalizer.driver.getPosX(DistanceUnit.MM));
        telemetry.addData("RPM Left", getRPMLeft());
        telemetry.addData("RPM Right", getRPMRight());
        telemetry.addData("RPM TPS Left", rpmToTicksPerSec(getRPMLeft()));
        telemetry.addData("RPM TPS Right", rpmToTicksPerSec(getRPMRight()));
        telemetry.update();
        updateTelemetry(telemetry);
    }
}
