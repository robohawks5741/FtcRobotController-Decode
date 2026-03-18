package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

@TeleOp(name="LLTest")
public class LLTest extends robot{
    public Pose2d limeLightBotpose;
    public double conversionRatio = 39.3701;
    @Override
    public void runOpMode() throws InterruptedException {
        super.runOpMode();
        //limelight.
        waitForStart();
        while (opModeIsActive()) {
            TelemetryPacket packet = new TelemetryPacket();
            packet.fieldOverlay().setStroke("#3F51B5");
            drive.setDrivePowers(new PoseVelocity2d(
                    new Vector2d(
                            -gamepad2.left_stick_y,
                            -gamepad2.left_stick_x
                    ),
                    -gamepad2.right_stick_x
            ));
            if(result.isValid() && result != null) {
                limeLightBotpose = new Pose2d(new Vector2d(result.getBotpose().getPosition().x*conversionRatio, result.getBotpose().getPosition().y*conversionRatio), result.getBotpose().getOrientation().getYaw(AngleUnit.RADIANS));
                drive.localizer.setPose(limeLightBotpose);
                telemetry.addData("BotPose MT1",result.getBotpose());
                telemetry.addData("2D BotPose MT1", limeLightBotpose);
                telemetry.addData("BotPose MT2",result.getBotpose_MT2());
                packet.put("x", drive.localizer.getPose().position.x);
                packet.put("y", drive.localizer.getPose().position.y);
                packet.put("BotPose MT1",result.getBotpose());
                packet.put("2D BotPose MT1", limeLightBotpose);
                packet.put("BotPose MT2",result.getBotpose_MT2());
                telemetry.update();
            }


            Drawing.drawRobot(packet.fieldOverlay(), drive.localizer.getPose());
            FtcDashboard.getInstance().sendTelemetryPacket(packet);
        }
    }
}
