package org.firstinspires.ftc.teamcode;

import static androidx.core.math.MathUtils.clamp;
import static java.lang.Math.abs;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import org.firstinspires.ftc.teamcode.subsystems.MecanumDrive;

import java.util.concurrent.TimeUnit;

@TeleOp(name="PID Tuning")
public class PIDTuning extends robot {
    ElapsedTime timer = new ElapsedTime();
    long lastChangedTime = 0;

    @Override
    public void runOpMode() throws InterruptedException {
        //DcMotorEx motor = hardwareMap.get(DcMotorEx.class, "motor");
        super.runOpMode();

      //  launcher.setVelocityPIDFCoefficients(0.4,0.001,0.3, 4);
        double Kp = 0.01;
        double Ki = 0;
        //double Kd = 0.000000045;
        double Kd = 0.4;
        double changingValue = 0;

        Pose2d beginPose;
        double turretYawRobot = 0.0;
        double turretYawRobot2 = 0.0;
        PID pid = new PID();
        double TX = 0;
        boolean beginPoseFound = false;
        Limelight3A limelight = hardwareMap.get(Limelight3A.class, "limelight");

        telemetry.setMsTransmissionInterval(10);

        limelight.pipelineSwitch(0);
        limelight.start();

        telemetry.update();
        MecanumDrive drive = new MecanumDrive(hardwareMap, new Pose2d(new Vector2d(0,0), 0));

        waitForStart();

        while (opModeIsActive()) {
            drive.updatePoseEstimate();
            result = limelight.getLatestResult();
            YawPitchRollAngles orientation = imu.getRobotYawPitchRollAngles();
            limelight.updateRobotOrientation(orientation.getYaw());
            colorLogger();

            if (gamepad1.left_bumper){
                hood.setPosition(clamp(gamepad1.left_stick_y, 0.25, 0.95));
            }

            if (timer.now(TimeUnit.SECONDS) > lastChangedTime + 1) {
                if (gamepad1.dpad_left) {
                    changingValue -= 1;
                    if (changingValue < 0) changingValue = 2;
                    lastChangedTime = timer.now(TimeUnit.SECONDS);
                } else if (gamepad1.dpad_right) {
                    changingValue += 1;
                    if (changingValue > 2) changingValue = 0;
                    lastChangedTime = timer.now(TimeUnit.SECONDS);
                }
            }

            double changeValue = (gamepad1.dpad_up ? 0.001 : 0) + (gamepad1.dpad_down ? -0.001 : 0);
            if (changingValue == 0) {
                Kp += changeValue;
            } else if (changingValue == 1) {
                Ki += changeValue;
            } else if (changingValue == 2) {
                Kd += changeValue;
            }

            if (result != null && result.isValid()) {
                TX = result.getTx();
                telemetry.addData("Target X", TX);
                double pidTarget = -pid.PIDControl(Kp, Ki, Kd, 0.0, limelight.getLatestResult().getTx());

                if (!gamepad1.right_bumper) {
                    telemetry.addData("PID Target", pidTarget);
                    turret1.setPower(pidTarget);
                } else {
                    turret1.setPower(-gamepad1.left_stick_x);
                }
            } else {
                pid.lastError = 0;
                pid.integralSum = 0;
                pid.time = new ElapsedTime();
                telemetry.addData("Limelight", "No Targets");
                if (!gamepad1.right_bumper){
                    turret1.setPower(-gamepad1.left_stick_x);
                } else {
                    turret1.setPower(0);
                }
            }

            if (abs(TX) < 4) {
                telemetry.addData("Target:", "Acquired");
            }

            if (gamepad2.circle) {
                limelight.pipelineSwitch(1);
                telemetry.addData("ID", result.getFiducialResults().get(0).getFiducialId());
            } else {
                limelight.pipelineSwitch(0);
            }

            drive.setDrivePowers(new PoseVelocity2d(
                    new Vector2d(
                            -gamepad1.left_stick_y,
                            -gamepad1.left_stick_x
                    ),
                    -gamepad1.right_stick_x
            ));

            if (result != null) {
                Pose3D botpose = result.getBotpose();
                Pose3D botposeMT2 = result.getBotpose_MT2();
                telemetry.addData("tx", result.getTx());
                //telemetry.addData("rz", result.getRz());
                telemetry.addData("ty", result.getTy());
                telemetry.addData("ta", result.getTa());
                telemetry.addData("Botpose", botpose.toString());
                telemetry.addData("BotposeMT2", botposeMT2.toString());

            }

            telemetry.addData("calculated angle to red", Math.atan2(drive.localizer.getPose().position.x-(-58.31), drive.localizer.getPose().position.y - (55.64)));
            if (result != null) {
                telemetry.addData("X", result.getBotpose().getPosition().x);
                telemetry.addData("Y", result.getBotpose().getPosition().y);
                telemetry.addData("TurretHeading", result.getBotpose().getOrientation().getYaw(AngleUnit.DEGREES));
                telemetry.addData("turretYaw3", result.getTx());
            }

            telemetry.addLine();
            if (changingValue == 0) {
                telemetry.addData("Currently changing", "Kp");
            } else if (changingValue == 1) {
                telemetry.addData("Currently changing", "Ki");
            } else if (changingValue == 2) {
                telemetry.addData("Currently changing", "Kd");
            }

            telemetry.addLine();
            telemetry.addData("Kp", Kp);
            telemetry.addData("Ki", Ki);
            telemetry.addData("Kd", Kd);

            telemetry.update();
            TelemetryPacket packet = new TelemetryPacket();
            packet.fieldOverlay().setStroke("#3F51B5");
            Drawing.drawRobot(packet.fieldOverlay(), drive.localizer.getPose());
            FtcDashboard.getInstance().sendTelemetryPacket(packet);
        }
    }
}