package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name="smallMove")
public class AutoBasic extends AutoBackBlue{
    @Override
    public void runOpMode() throws InterruptedException {
        waitForStart();
        if (opModeIsActive()) {
            Actions.runBlocking(new SequentialAction(
                    drive.actionBuilder(beginPos)
                            .lineToX(45)
                            .build()
            ));
        }
    }
}
