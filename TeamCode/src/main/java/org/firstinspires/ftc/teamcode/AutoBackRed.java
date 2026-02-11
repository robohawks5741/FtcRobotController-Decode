package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name = "AutoBackRed", group = "Robot")
public class AutoBackRed extends AutoBackBlue{
    public static Params PARAMS = AutoBackBlue.PARAMS;
    @Override
    public void runOpMode() throws InterruptedException {
        //PARAMS.beginPosX = 62;
        //Flips Y values and heading for red side of field
        PARAMS.beginPosY *= -1;
        PARAMS.targetY *= -1;
        PARAMS.targetHeading *= -1;
        super.runOpMode();
    }
}
