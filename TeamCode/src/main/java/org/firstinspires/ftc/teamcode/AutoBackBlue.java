package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.AccelConstraint;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.AngularVelConstraint;
import com.acmerobotics.roadrunner.MinVelConstraint;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Pose2dDual;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.PoseVelocity2dDual;
import com.acmerobotics.roadrunner.ProfileAccelConstraint;
import com.acmerobotics.roadrunner.RaceAction;
import com.acmerobotics.roadrunner.Rotation2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.Time;
import com.acmerobotics.roadrunner.Trajectory;
import com.acmerobotics.roadrunner.TranslationalVelConstraint;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.TranslationalVelConstraint;
import com.acmerobotics.roadrunner.TurnConstraints;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.VelConstraint;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.teamcode.MecanumDrive;

import java.util.Arrays;

@Autonomous(name = "AutoBackBlue", group = "Robot")
public class AutoBackBlue extends LinearOpMode {

    @Override
    public void runOpMode() {
        telemetry.addData("Status:", " Initialized");
        telemetry.update();
        //super.runOpMode();
        Pose2d beginPose = new Pose2d(0, 0, Math.toRadians(0));
        MecanumDrive drive = new MecanumDrive(hardwareMap, beginPose);
        //PinpointLocalizer pinpoint = new PinpointLocalizer(hardwareMap, 0.0007669904, beginPose);
        waitForStart();
        if (opModeIsActive()) {
            if (isStopRequested()) return;
            telemetry.addData("Status: ", "Running");
            drive.localizer.setPose(beginPose);
          /*  Actions.runBlocking(
                    drive.actionBuilder(drive.localizer.getPose())
                            .lineToYConstantHeading(10)
                            // .strafeTo(new Vector2d(0, 0))
                            .turn(Math.toRadians(90))
                            .build()
            );*/
            drive.localizer.update();
            beginPose = drive.localizer.getPose();
            double time = 0;
            Actions.runBlocking(new ParallelAction(
                    drive.actionBuilder(beginPose)
                            .splineToConstantHeading(new Vector2d(-50, 0), 0)
                            .build()));
            telemetry.addData("Status: ", "Done");
            telemetry.addData("PPX", drive.localizer.getPose().position.x);
            telemetry.addData("PPY", drive.localizer.getPose().position.y);
            telemetry.addData("PPRot", drive.localizer.getPose().heading.toDouble());
            telemetry.update();
            while (opModeIsActive()) {
                if (time > 20) {
                    telemetry.addData("Status: ", "Idle - Telemetry");
                    telemetry.addData("PPX", drive.localizer.getPose().position.x);
                    telemetry.addData("PPY", drive.localizer.getPose().position.y);
                    telemetry.addData("PPRot", drive.localizer.getPose().heading.toDouble());
                    telemetry.update();
                }
            }
        }
    }
}
