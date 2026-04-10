package org.firstinspires.ftc.teamcode.opmodes.auto;

import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.robot;

@Autonomous(name = "Aidan Basic Auto", group = "Robot")
public class AidanAuto extends robot {
    private static ElapsedTime timer = new ElapsedTime();

    @Override
    public void runOpMode() {
        telemetry.update();
        waitForStart();
        timer.reset();
        while (opModeIsActive()) {
            if (timer.time() >= 1.0) {
                drive.setDrivePowers(new PoseVelocity2d(
                        new Vector2d(0, 0),
                        0
                ));
                break;
            }
            drive.setDrivePowers(new PoseVelocity2d(
                    new Vector2d(1, 0),
                    0
            ));
        }
    }
}