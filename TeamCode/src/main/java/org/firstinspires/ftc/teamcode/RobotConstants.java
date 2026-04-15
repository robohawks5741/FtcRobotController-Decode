package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.Vector2d;

/**
 * All tunable robot constants in one place.
 * Annotated with @Config so values can be live-tuned via FTC Dashboard.
 */
@Config
public class RobotConstants {
    @Config
    public static class LLTEST {
        public static boolean COMPLEX = false;
    }
    @Config
    public static class TurretPID {
        public static double kP = 16.8;
        public static double kI = 0.21;
        public static double kD = 2.9;
        public static double zeroOffset = 8.5;  // degrees
        public static double gearRatio = 4.167;
        public static double kF = 0.1;

    }
    @Config
    public static class IndexerPID {
        public static double kP = 0.000000001;
        public static double kI = 0.00000000000000001;
        public static double kD = 0.00000003;
        public static double kD_loaded = 0.00000003;
        public static double kD_unloaded = 0.00000002;
    }
    @Config
    public static class LauncherPID {
        public static double kP = 22;
        public static double kI = 0.3;
        public static double kD = 0;
        public static double kF = 12;
    }
    @Config
    public static class DriveControl {
        public static double kP = 0.01;
        public static double kI = 0.0000001;
        public static double kD = 0.4;
    }
    @Config
    public static class FieldPositions {
        public static double redGoalX = -62;
        public static double redGoalY = 64;
        public static double blueGoalX = -62;
        public static double blueGoalY = -64;

        public static Vector2d redGoalVector = new Vector2d(redGoalX, redGoalY);
        public static Vector2d blueGoalVector = new Vector2d(blueGoalX, blueGoalY);

    }
    @Config
    public static class LauncherCalibration {
        public static double launcherM = 20;    // RPM per inch slope
        public static double launcherB = 950;      // RPM minimum (y-intercept)
        public static double hoodM = 0.7 / 145.0;  // hood position per inch slope
        public static double hoodB = 0.25;          // hood minimum position
        public static double hoodMin = 0.25;
        public static double hoodMax = 0.95;
        public static int teleopPower = 4000;       // RPM for teleop
        public static int autoPower = 2150;          // RPM for auto
        public static double teleOpIndexPower = 0.3;
        public static double autoIndexPower = 1;
        public static double normalRunIndexTime = 2.4;
        public static double autoRunIndexTime = 1.9;
        public static double shortRunIndexTime = 0.7;
    }
    @Config
    public static class StartPositions {
        public static double beginPosX = 65;
        public static double beginPosY = 11;
        public static double beginHeading = Math.toRadians(180);
    }
    @Config
    public static class AutoPaths {
        public static double targetX = -53;
        public static double targetY = 25;
        public static double row1X = 34.5;
        public static double row1Y = 68;
        public static double row2X = 10;
        public static double row2Y = 69;
        public static double row3X = -13;
        public static double row3Y = 62;
        public static double backToY = 25;
        public static double targetHeading = 110;
        public static double parkX = -24;
        public static double parkY = 24;
    }
    @Config
    public static class IndexPositions {
        public static double pos0 = 0.0;
        public static double pos1 = 0.21;
        public static double pos2 = 0.35;
        public static double pos3 = 0.60;
        public static double pos4 = 0.69;
        public static double pos5 = 1.0;
    }
    @Config
    public static class ColorThresholds {
        public static double minIntensity = 175;
        public static double minDifference = 100;
    }
    @Config
    public static class Misc {
        public static double conversionRatio = 39.3701;  // meters to inches
        public static double llTurretRadius = 6;          // inches
        public static double ticksPerRev = 384.5;
        public static double turretRadiusAtLimeLight = 6.25; //Inches
        public static double turretCenterToRobotCenter = 3; //Inches
        public static double ticksPerDegree = 1.068;
        public static double turretRatio = 4;
        public static double checkInterval = 0.2;
        public static double distTolerance = 20;
        public static double tickUpdateMaxTicks = 8;
        public static double tickUpdateHeadingMaxTicks = 15;
    }
}
