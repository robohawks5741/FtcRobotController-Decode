package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name="##Tele-Blue")
public class teleOpBlue extends indexLaunchTest{
    @Override
    public void runOpMode() throws InterruptedException {
        isTeleOpRed = false;
        isRedAlliance = false;

        //PARAMS.modifier = 1;
        super.runOpMode();
    }
}
