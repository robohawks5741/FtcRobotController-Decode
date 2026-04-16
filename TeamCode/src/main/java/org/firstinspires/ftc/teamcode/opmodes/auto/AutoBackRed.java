package org.firstinspires.ftc.teamcode.opmodes.auto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name = "AutoBackRed", group = "Robot")
public class AutoBackRed extends AutoBackBlue{
    public static Params PARAMS = AutoBackBlue.PARAMS;
    @Override
    public void runOpMode() throws InterruptedException {
        //PARAMS.beginPosX = 62;
        //Flips Y values and heading for red side of field
        redOverride = true;
        isRedAlliance = true;
        super.runOpMode();
    }
}
