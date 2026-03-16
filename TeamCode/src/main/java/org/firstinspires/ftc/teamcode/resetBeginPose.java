package org.firstinspires.ftc.teamcode;

public class resetBeginPose extends robot{
    @Override
    public void runOpMode() throws InterruptedException {
        teleOpBeginPose = null;
        while (opModeIsActive()){
            telemetry.addData("teleOpBeginPose = null", "");
            if (isStopRequested()) {
                break;
            }
        }
    }
}
