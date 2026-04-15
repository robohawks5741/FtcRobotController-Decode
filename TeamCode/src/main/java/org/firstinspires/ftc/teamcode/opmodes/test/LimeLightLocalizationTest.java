package org.firstinspires.ftc.teamcode.opmodes.test;
import static java.lang.Math.PI;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.RobotConstants;
import org.firstinspires.ftc.teamcode.robot;
import org.firstinspires.ftc.teamcode.util.Drawing;

@TeleOp(name="#LL Localization Test")
public class LimeLightLocalizationTest extends robot{
    boolean COMPLEX = RobotConstants.LLTEST.COMPLEX;

    @Override
    public void runOpMode() throws InterruptedException {
        super.runOpMode();
        while (opModeInInit()) {
            if (limelight.isRunning()) {
                result = limelight.getLatestResult();
            }
            if (result.isValid() && result != null) {
                drive.localizer.setPose(new Pose2d(new Vector2d(drive.localizer.getPose().position.x, drive.localizer.getPose().position.y),result.getBotpose().getOrientation().getYaw(AngleUnit.RADIANS)+PI));
                limelight.updateRobotOrientation(Math.toDegrees(drive.localizer.getPose().heading.toDouble()));
            }
        }
        if (COMPLEX) {
            drive.localizer.setPose(new Pose2d(new Vector2d(drive.localizer.getPose().position.x, drive.localizer.getPose().position.y), result.getBotpose().getOrientation().getYaw(AngleUnit.RADIANS)));
        }else{
            drive.localizer.setPose(new Pose2d(new Vector2d(drive.localizer.getPose().position.x, drive.localizer.getPose().position.y), result.getBotpose().getOrientation().getYaw(AngleUnit.RADIANS)));
        }

        waitForStart();
        while (opModeIsActive()) {
            COMPLEX = RobotConstants.LLTEST.COMPLEX;
            result = limelight.getLatestResult();
            drive.updatePoseEstimate();
            drive.localizer.update();
            TelemetryPacket packet = new TelemetryPacket();
            if (result.isValid() && result != null) {
                if (COMPLEX) {
                    //drive.localizer.setPose(new Pose2d(new Vector2d(drive.localizer.getPose().position.x, drive.localizer.getPose().position.y), result.getBotpose().getOrientation().getYaw(AngleUnit.RADIANS)));
                    drive.localizer.setPose(updatePoseFromLimeLight());
                } else {
                    Pose2d rawLlPose = new Pose2d(
                            result.getBotpose().getPosition().x * conversionRatio,
                            result.getBotpose().getPosition().y * conversionRatio,
                            result.getBotpose().getOrientation().getYaw(AngleUnit.RADIANS)
                    );
                    limeLightBotpose = rawLlPose;
                    drive.localizer.setPose(rawLlPose);
                }
                packet.put("Result valid and not null:", true);
            } else {
                packet.put("Result valid and not null:", false);

            }
            if (gamepad2.right_trigger>0) {
                setTurretPosition(getTargetTurretPosition()-1*gamepad2.right_trigger);
            } else if (gamepad2.left_trigger>0) {
                setTurretPosition(getTargetTurretPosition()+1*gamepad2.left_trigger);
            }
            packet.put("TurretTarget", getTargetTurretPosition());
            packet.put("TurretCurrent", getTurretPosition());

            drive.setDrivePowers(new PoseVelocity2d(
                    new Vector2d(
                            -gamepad2.left_stick_y,
                            -gamepad2.left_stick_x
                    ),
                    -gamepad2.right_stick_x
            ));
            robot.PARAMS.localizerX = drive.localizer.getPose().position.x;
            robot.PARAMS.localizerY = drive.localizer.getPose().position.y;
            blueGoalHeading = Math.toDegrees(Math.atan2(blueGoalY -PARAMS.localizerY, blueGoalX -PARAMS.localizerX));
            blueGoalDistance = Math.hypot(blueGoalX -PARAMS.localizerX, blueGoalY -PARAMS.localizerY);
            redGoalHeading = Math.toDegrees(Math.atan2(redGoalY -PARAMS.localizerY, redGoalX -PARAMS.localizerX));
            redGoalDistance = Math.hypot(redGoalX -PARAMS.localizerX, redGoalY -PARAMS.localizerY);
            telemetry.update();
            packet.fieldOverlay().setStroke("#3F51B5");
            packet.fieldOverlay().setStroke("#3F51B5");
            packet.put("x", drive.localizer.getPose().position.x);
            packet.put("y", drive.localizer.getPose().position.y);
            packet.put("Head", Math.toDegrees(drive.localizer.getPose().heading.toDouble()));
            packet.put("HeadRad", (drive.localizer.getPose().heading.toDouble()));
            if (limeLightBotpose != null) {
                packet.put("LLX", limeLightBotpose.position.x);
                packet.put("LLY", limeLightBotpose.position.y);
                packet.put("LLheading", Math.toDegrees(limeLightBotpose.heading.toDouble()));
            }
            Drawing.drawRobot(packet.fieldOverlay(), drive.localizer.getPose());
            FtcDashboard.getInstance().sendTelemetryPacket(packet);
        }
    }
}
