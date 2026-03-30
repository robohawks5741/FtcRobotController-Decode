# Robot Code Guide - FTC 2026 Decode Season

## Project Structure

```
teamcode/
  robot.java               Base class - all hardware init, subsystem methods, auto actions
  RobotConstants.java       All tunable constants (@Config for FTC Dashboard)
  opmodes/
    auto/
      AutoBackBlue.java     Blue auto (back position) - main auto routine
      AutoBackRed.java      Red auto (extends AutoBackBlue, flips alliance)
      AutoBlueFront.java    Blue auto (front position)
      AutoBasic.java        Minimal auto for testing
    teleop/
      indexLaunchTest.java   Main teleop (##Tele-Red) - full turret/launch/intake
      teleOpBlue.java        Blue teleop (extends indexLaunchTest, flips alliance)
    test/
      turretTest.java        Turret + Limelight PID testing
      PIDTuning.java         General PID tuning with live dpad adjustments
      LLTest.java            Limelight pose visualization on Dashboard
      motorTester.java       Raw motor velocity/power testing
      launchTester.java      Launcher + hood testing
      resetBeginPose.java    Clears teleOpBeginPose between matches
      newMain.java           Legacy teleop with AprilTag turn-to-goal
      JVMain.java            Simplified JV teleop (no turret/indexer)
      test.java              Basic mecanum movement test
  subsystems/
    MecanumDrive.java        RoadRunner mecanum drive + trajectory following
    PinpointLocalizer.java   GoBilda Pinpoint odometry wrapper
    TwoDeadWheelLocalizer.java
    ThreeDeadWheelLocalizer.java
  util/
    PID.java                 PID controller with separate turret/indexer/general instances
    AprilTag.java            AprilTag vision processor setup
    Drawing.java             FTC Dashboard field overlay drawing
    Localizer.java           Localizer interface
    OTOSLocalizer.java       SparkFun OTOS localizer (alternative)
    TankDrive.java           Tank drive variant (unused)
  messages/                  Telemetry message classes for Dashboard
  tuning/                    RoadRunner tuning opmodes
```

## Hardware Map

| Name | Type | Purpose |
|------|------|---------|
| `fldrive`, `frdrive`, `bldrive`, `brdrive` | DcMotorEx | Mecanum drive motors |
| `launcher` | DcMotorEx | Flywheel launcher (velocity-controlled) |
| `intake` | DcMotorEx | Ball intake roller |
| `turret1` | CRServo | Turret rotation (continuous) |
| `turretFB` | AnalogInput | Turret position feedback (potentiometer) |
| `index` | CRServo | Indexer wheel rotation (continuous) |
| `indexFB` | AnalogInput | Indexer position feedback (potentiometer) |
| `hood` | Servo | Launch angle adjustment (0.25 - 0.95 range) |
| `launchFeedL`, `launchFeedR` | CRServo | Ball feed into launcher |
| `liftL`, `liftR` | DcMotorEx | Lift motors |
| `color1`, `color2` | ColorSensor | Ball color detection (purple vs green) |
| `light1`, `light2`, `light3` | Servo | Index slot status LEDs |
| `limelight` | Limelight3A | Vision system for AprilTag tracking + pose |
| `imu` | IMU | Heading reference (logo left, USB up) |

## How the Robot Works End-to-End

### Startup

1. An opmode (e.g. `indexLaunchTest`) calls `super.runOpMode()` which runs `robot.runOpMode()`
2. `robot.runOpMode()` does all hardware initialization:
   - Maps all motors, servos, sensors from `hardwareMap`
   - Sets alliance modifier (1 for red, -1 for blue) and flips all Y coordinates and headings
   - Creates three PID controller instances (general, indexer, turret)
   - Configures launcher motor PIDF from `RobotConstants.LauncherPID`
   - Initializes PinpointLocalizer and IMU
   - Starts Limelight streaming
   - Waits for start
   - Creates `MecanumDrive` with the starting pose

### Pose Tracking

The robot tracks its position on the field using two systems:

**Primary: PinpointLocalizer (via MecanumDrive.localizer)**
- GoBilda Pinpoint odometry pod reads wheel encoders + IMU heading
- Updated every loop via `drive.localizer.update()` and `drive.updatePoseEstimate()`
- Returns `Pose2d` (x inches, y inches, heading radians)

**Secondary: Limelight 3A**
- Detects AprilTags on the field to get absolute field position
- `updatePoseFromLimeLight()` syncs the localizer pose when:
  - The result is valid (tags visible)
  - The turret is close to its target angle (within 3 degrees of zero offset)
- Converts from meters to inches using `conversionRatio` (39.3701)
- The IMU yaw is fed to Limelight via `limelight.updateRobotOrientation()` for MT2 pose

### Turret Tracking

The turret points a launcher at the goal. It can be auto-tracked or manually controlled.

**Auto-tracking (`updateTurretTracking`)**:
1. Gets current robot position from localizer
2. Computes field-frame angle from robot to goal: `atan2(goalY - botY, goalX - botX)`
3. Subtracts robot heading to get robot-relative angle
4. Normalizes via `atan2(sin, cos)` to stay in [-180, 180] degrees
5. Sets `targetTurretAngle` to this value
6. Also updates `redGoalDistance`/`blueGoalDistance` for hood/velocity calculations
7. Resets turretPID before each computation (new target each loop)

**Turret servo control (`setTurretPosition`)**:
1. Reads turret potentiometer: `(voltage / 3.3) * 360` degrees
2. Tracks cumulative rotations (detects 280-degree jumps as wraparound)
3. Computes total position: `(rawAngle + rotations * 360) / 4.167` (gear ratio)
4. Adds `turretZeroOffset` (8.5 degrees) to the target
5. Runs turretPID to compute servo power
6. Clamps power to [-1, 1] and sets CRServo

**The gear ratio (4.167)** accounts for the motor gear driving the turret. The feedback sensor reads motor-side angle, so dividing by 4.167 converts to turret-side angle. The target is in turret-side degrees, and the feedback is also converted to turret-side degrees, so they match.

### Launcher System

**Distance-based calibration (`launcherAngleVelocity`)**:
- Hood position: `hoodM * distance + hoodB` (linear fit, clamped 0.25-0.95)
- Launcher RPM: `launcherM * distance + launcherB` (linear fit)
- Constants in `RobotConstants.LauncherCalibration`

**Motor velocity control (`setLaunchRPM`)**:
- Uses built-in motor velocity PIDF (set during init)
- Converts RPM to degrees/sec: `rpm / 19.1`
- Motor runs in reverse (negative velocity)

### Indexer System

The indexer is a rotating wheel with 3 arms that carry balls between intake and launch positions.

**Positions (0-5)**: Each position is a fraction of a full rotation (0.0 to 1.0), stored in `RobotConstants.IndexPositions`. The 6 positions cycle through which arm is at intake vs launch.

**Position control (`setIndexPosition`)**:
1. Reads indexer potentiometer and converts to degrees
2. Tracks cumulative rotations (same wraparound logic as turret)
3. Uses indexPID to compute servo power
4. Applies 2-degree deadband to prevent servo whine at target
5. Clamps power to [-0.5, 0.5]

**Artifact tracking**:
- `artifacts` list holds 3 slots: `[slot0, slot1, slot2]`
- Values: 0=empty, 1=purple, 2=green
- `colorLogger()` reads color sensors each loop and updates slots based on current index position and whether intake/launcher is active
- Light signals update to show slot contents (purple=0.69, green=0.51, empty=0.28)

### Launch Cycle (`newLaunchCycle`)

This is a blocking action that runs the full launch sequence:

1. Turns on feed and intake
2. Sets launcher RPM (auto power or teleop distance-based power)
3. Waits for launcher to reach target speed (within 100 RPM)
4. Runs indexer at full speed (auto) or half speed (teleop) for 1.9 seconds
5. If teleop: continuously auto-tracks turret, updates hood/RPM, allows driving
6. Falls back to timeout at 10-12 seconds if speed never reached
7. On exit: stops feed, clears artifacts, resets indexer position

### Autonomous Sequence

`AutoBackBlue` runs a fixed sequence:

1. **Preload launch**: Drive to target position (rowSelectAuto 4), launch balls (newLaunchCycle)
2. **Row 2 intake**: Drive to row 2 with intake on, collect balls, drive back to target, launch
3. **Row 3 intake**: Same as above for row 3
4. **Save pose**: `sendAutoEndPose` captures final position for teleop handoff

The `rowSelectAuto` action builds RoadRunner trajectories:
- Uses spline paths for smooth motion
- Toggles intake on/off at waypoints
- Coordinates heading changes for optimal approach angles
- All coordinates flipped by `modifier` for red vs blue alliance

### Alliance Handling

Setting `isRedAlliance` flips the field:
- `modifier = 1` (red) or `-1` (blue)
- All Y coordinates multiplied by modifier
- Heading flipped by modifier
- Goal position selected accordingly (redGoal vs blueGoal)
- `teleOpBlue` simply sets `isTeleOpRed = false` and calls super

### Auto-to-Teleop Handoff

1. At end of auto, `sendAutoEndPose` saves `drive.localizer.getPose()` to static `teleOpBeginPose`
2. Teleop reads `teleOpBeginPose` and calls `drive.localizer.setPose(teleOpBeginPose)` to resume from where auto left off
3. If `teleOpBeginPose` is null (no auto ran), uses default `beginPos`
4. `resetBeginPose` opmode clears this to null if needed between matches

## Teleop Controls

### Gamepad 1 (Operator)
| Control | Action |
|---------|--------|
| Right bumper + right stick X | Manual turret adjust |
| Left stick click | Turret to 45 degrees |
| Share | Center turret (0 degrees) |
| Right trigger | Spin up launcher |
| Left trigger | Feed balls to launcher |
| Left bumper + left stick Y | Manual hood position |
| Right stick click | Run full launch cycle |
| Square / Triangle / Circle | Index to position 0/2/4 |
| Dpad up/down | Lift up/down |
| Dpad left/right | Turret rotation offset adjust |

### Gamepad 2 (Driver)
| Control | Action |
|---------|--------|
| Left stick | Drive (forward/strafe) |
| Right stick X | Rotate |
| Right trigger | Intake forward |
| Left trigger | Intake reverse |
| Circle / Square | Index increment/decrement by 2 |

## Tuning

### FTC Dashboard
All values in `RobotConstants` are live-tunable via FTC Dashboard at `192.168.43.1:8080/dash`. Changes take effect immediately without redeploying.

### Key Values to Tune
- **Turret PID**: `RobotConstants.TurretPID.kP/kI/kD` - adjust if turret oscillates or is sluggish
- **Launcher calibration**: `LauncherCalibration.launcherM/launcherB` - linear fit of RPM vs distance
- **Hood calibration**: `LauncherCalibration.hoodM/hoodB` - linear fit of hood angle vs distance
- **Color thresholds**: `ColorThresholds.minIntensity/minDifference` - adjust for lighting conditions
- **Auto paths**: `AutoPaths.*` - field coordinates for intake rows and target position

### PID Tuning Tips
- The turret PID resets every loop because a new target angle is computed each cycle
- The indexer PID has extremely small gains because the continuous servo needs gentle control
- The launcher uses the motor's built-in PIDF (not the custom PID class)
