package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.config.Config;

/**
 * All tunable robot constants in one place.
 * Annotated with @Config so values can be live-tuned via FTC Dashboard.
 */
@Config
public class RobotConstants {

    public static class TurretPID {
        public static double kP = 0.017;
        public static double kI = 0.000000001;
        public static double kD = 0.00000003;
        public static double zeroOffset = 8.5;  // degrees
        public static double gearRatio = 4.167;
    }

    public static class IndexerPID {
        public static double kP = 0.000000001;
        public static double kI = 0.00000000000000001;
        public static double kD = 0.00000003;
        public static double kD_loaded = 0.00000003;
        public static double kD_unloaded = 0.00000004;
    }

    public static class LauncherPID {
        public static double kP = 22;
        public static double kI = 0.3;
        public static double kD = 0;
        public static double kF = 12;
    }

    public static class DriveControl {
        public static double kP = 0.01;
        public static double kI = 0.0000001;
        public static double kD = 0.4;
    }

    public static class FieldPositions {
        public static double redGoalX = -62;
        public static double redGoalY = 64;
        public static double blueGoalX = -62;
        public static double blueGoalY = -64;
    }

    public static class LauncherCalibration {
        public static double launcherM = 12.25;    // RPM per inch slope
        public static double launcherB = 1500;      // RPM minimum (y-intercept)
        public static double hoodM = 0.7 / 145.0;  // hood position per inch slope
        public static double hoodB = 0.25;          // hood minimum position
        public static double hoodMin = 0.25;
        public static double hoodMax = 0.95;
        public static int teleopPower = 4000;       // RPM for teleop
        public static int autoPower = 2150;          // RPM for auto
    }

    public static class StartPositions {
        public static double beginPosX = 65;
        public static double beginPosY = 11;
        public static double beginHeading = Math.toRadians(180);
    }

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
    }

    public static class IndexPositions {
        public static double pos0 = 0.0;
        public static double pos1 = 0.21;
        public static double pos2 = 0.35;
        public static double pos3 = 0.60;
        public static double pos4 = 0.69;
        public static double pos5 = 1.0;
    }

    public static class ColorThresholds {
        public static double minIntensity = 175;
        public static double minDifference = 100;
    }

    public static class Misc {
        public static double conversionRatio = 39.3701;  // meters to inches
        public static double llTurretRadius = 6;          // inches
        public static double ticksPerRev = 384.5;
    }
}
