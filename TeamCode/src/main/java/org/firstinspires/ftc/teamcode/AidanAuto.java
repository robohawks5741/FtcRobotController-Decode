package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.util.ElapsedTime;

@Autonomous(name = "Aidan Auto", group = "Robot")
public class AidanAuto extends robot {
    private static ElapsedTime timer = new ElapsedTime();

    @Override
    public void runOpMode() {
        telemetry.update();
        waitForStart();
        timer.reset();
        while (opModeIsActive()) {
            if (timer.time() > 1) return;
            drive.setDrivePowers(new PoseVelocity2d(
                    new Vector2d(
                            1,
                            0
                    ),
                    0
            ));
        }
    }
}