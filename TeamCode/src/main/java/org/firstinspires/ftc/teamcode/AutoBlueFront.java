package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name = "AutoFrontBlue", group = "Robot")

public class AutoBlueFront extends AutoBackBlue{
    public static Params PARAMS = AutoBackBlue.PARAMS;
    @Override
    public void runOpMode() throws InterruptedException {
        //PARAMS.beginPosX = 62;
        //Flips Y values and heading for red side of field
        beginPos = new Pose2d(new Vector2d(-45, 35), Math.toRadians(-60));
        super.runOpMode();
    }
}
