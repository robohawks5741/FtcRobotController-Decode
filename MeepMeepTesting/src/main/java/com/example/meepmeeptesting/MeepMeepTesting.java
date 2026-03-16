package com.example.meepmeeptesting;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;

import org.rowlandhall.meepmeep.MeepMeep;
import org.rowlandhall.meepmeep.roadrunner.DefaultBotBuilder;
import org.rowlandhall.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

import java.awt.Image;
import java.io.File;
import java.io.IOException;

import javax.imageio.ImageIO;

public class MeepMeepTesting {
    public static class Params {
        public double beginPosX = 60;
        public double beginPosY = 11;
        public double targetX = -21;
        public double targetY = 22;
        public double row1X = 34.5;
        public double row1Y = 60;
        public double row2X = 14;
        public double row2Y = 60;
        public double row3X = -12;
        public double row3Y = 53;
        public double endX = 30;
        public double endY = -20;
        public double targetHeading = 135;
    }
    public static Params PARAMS = new Params();
    public static void main(String[] args) {

        MeepMeep meepMeep = new MeepMeep(800);

        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(65, 40, Math.toRadians(180), Math.toRadians(180), 15)
                .followTrajectorySequence(drive -> drive.trajectorySequenceBuilder(
                                new Pose2d(PARAMS.row1X, PARAMS.targetY, Math.toRadians(90))) // Removed extra ) here
                       // .lineToSplineHeading(new Pose2d(PARAMS.beginPosX, PARAMS.beginPosY, Math.toRadians(180)))

                        //launch preload
                        .splineToSplineHeading(new Pose2d(PARAMS.targetX, PARAMS.targetY, Math.toRadians(PARAMS.targetHeading)), Math.toRadians(175))

                        //row 1
                        .splineToSplineHeading(new Pose2d(PARAMS.row1X, PARAMS.targetY, Math.toRadians(90)), Math.toRadians(90))
                        .splineToSplineHeading(new Pose2d(PARAMS.row1X, PARAMS.row1Y, Math.toRadians(90)), Math.toRadians(90))
                        .splineToSplineHeading(new Pose2d(PARAMS.row1X+5, PARAMS.targetY+5, Math.toRadians(180)), Math.toRadians(0))
                        .splineToSplineHeading(new Pose2d(PARAMS.targetX, PARAMS.targetY, Math.toRadians(PARAMS.targetHeading)), Math.toRadians(180))

                        //row 2
                        .splineToSplineHeading(new Pose2d(PARAMS.row2X, PARAMS.targetY, Math.toRadians(90)), Math.toRadians(90))
                        .splineToSplineHeading(new Pose2d(PARAMS.row2X, PARAMS.row2Y, Math.toRadians(90)), Math.toRadians(90))
                        .splineToSplineHeading(new Pose2d(PARAMS.row2X, PARAMS.targetY+5, Math.toRadians(90)), Math.toRadians(90))
                        .splineToSplineHeading(new Pose2d(PARAMS.targetX, PARAMS.targetY, Math.toRadians(PARAMS.targetHeading)), Math.toRadians(175))

                       /* //row 3
                        .splineToSplineHeading(new Pose2d(PARAMS.row3X, PARAMS.targetY, Math.toRadians(90)), Math.toRadians(90))
                        .splineToSplineHeading(new Pose2d(PARAMS.row3X, PARAMS.row3Y, Math.toRadians(90)), Math.toRadians(90))
                        .splineToSplineHeading(new Pose2d(PARAMS.row3X, PARAMS.targetY+5, Math.toRadians(90)), Math.toRadians(90))
                        .splineToSplineHeading(new Pose2d(PARAMS.targetX, PARAMS.targetY, Math.toRadians(PARAMS.targetHeading)), Math.toRadians(175))
*/
                        //go to empty area for leave points
                        .splineToSplineHeading(new Pose2d(PARAMS.row2X, PARAMS.targetY, Math.toRadians(90)), Math.toRadians(175))

                       /* .splineToSplineHeading(new Pose2d(PARAMS.row1X, PARAMS.row1Y, Math.toRadians(90)), Math.toRadians(90))
                        .splineToSplineHeading(new Pose2d(PARAMS.row1X, PARAMS.targetY + 5, Math.toRadians(90)), Math.toRadians(90))
                        .splineToSplineHeading(new Pose2d(PARAMS.targetX, PARAMS.targetY, Math.toRadians(PARAMS.targetHeading)), Math.toRadians(175))
                        */
                        .build()


                );

             //   );





        Image img = null;
        try { img = ImageIO.read(new File("C:\\GitHub\\FtcRobotController-DecodeTest\\MeepMeepTesting\\field-2025-official.png")); }
        catch(IOException e) {}

     //   meepMeep.setBackground(img)

        meepMeep.setBackground(img)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }
}