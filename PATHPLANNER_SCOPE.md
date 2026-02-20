# PathPlanner Integration Scope of Work

## Understanding the Camera Branch

The camera branch adds three things working together:

### 1. NavX Gyro + Wheel Odometry (`candrivesubsystem.py`)

The camera branch adds a NavX IMU and uses `DifferentialDriveOdometry` to track the robot's position on the field. Every loop cycle, it fuses gyro heading + encoder positions to estimate where the robot is (`Pose2d`). Without this, PathPlanner can't work at all — it needs to know where the robot is.

### 2. `VisionCamera`

A thin wrapper over Limelight's NetworkTables API. It reads/writes all the Limelight keys and monitors a heartbeat to detect if the camera goes stale. It also exposes the MegaTag2 localization interface when `addLocalizer()` is called.

### 3. `VisionLocalizer`

Fuses Limelight's **MegaTag2** pose estimate into the drivetrain's odometry. MegaTag2 works like this:

- You tell the Limelight the robot's current heading (from the NavX)
- The Limelight sees AprilTags on the field and computes the robot's absolute field position
- `VisionLocalizer` takes that absolute position and blends it toward the odometry position using a **learning rate** (e.g. 0.3 means "move 30% of the way toward the vision estimate each frame")

This is a drift correction mechanism. Wheel encoders accumulate error over time; vision snaps you back toward ground truth. For PathPlanner, this directly improves how accurately the robot follows paths.

---

## Bugs in the Camera Branch to Know About

Before merging, there are real bugs:

1. **CANcoder IDs clash with TalonFX IDs** — `CANcoder(LEFT_LEADER_ID)` uses ID 1, same as the TalonFX motor. They'd conflict on the CAN bus. The fix is to either use the TalonFX's internal encoder (simpler, no extra hardware) or give the CANcoders their own unique IDs.

2. **`VisionLocalizer` calls methods that don't exist on the drivetrain** — It calls `self.drivetrain.getHeading()`, `getTurnRate()`, and `adjustOdometry()`, but the camera branch's drivetrain only has `getGyroHeading()`. These would crash at runtime.

3. **Logic bug in `initEnabledChooser`** — There's an unreachable `print` statement after a `return` in the alliance color check.

4. **Camera branch reverted to `DutyCycleOut`** — The camera branch's `driveArcade` switched back from `VelocityVoltage` to `DutyCycleOut` (% output). PathPlanner needs voltage-based control, so the current `pid` branch's velocity control approach is actually closer to what you need.

---

## Scope of Work for PathPlanner

### 1. Add Required Physical Constants (`constants.py`)

PathPlanner needs real measurements from the robot. Measure and add:

```
TRACK_WIDTH_METERS        # distance between left and right wheels
WHEEL_RADIUS_METERS       # wheel radius
GEAR_RATIO                # motor rotations per wheel rotation
MAX_SPEED_MPS             # for path constraints
MAX_ACCEL_MPS2            # for path constraints
kS, kV, kA               # drivetrain characterization constants (from SysId)
```

### 2. Upgrade `CANDriveSubsystem` — The Biggest Change

This is the core of the work. The drivetrain needs all of these:

- **Gyro**: Merge in the NavX from the camera branch
- **Wheel encoders**: Use the TalonFX's built-in position/velocity signals (not CANcoders — avoids the ID conflict) converted to meters using gear ratio + wheel circumference
- **`getWheelSpeeds()`**: Returns `DifferentialDriveWheelSpeeds` in m/s — PathPlanner needs this for feedback control
- **`driveVolts(leftVolts, rightVolts)`**: PathPlanner drives the robot by sending voltage commands (feedforward + PID output). The current RPS-based `VelocityVoltage` control needs a voltage passthrough mode alongside it.
- **`DifferentialDriveKinematics`**: Instantiated with track width, used to convert `ChassisSpeeds` ↔ `WheelSpeeds`
- **Switch from `DifferentialDriveOdometry` to `DifferentialDrivePoseEstimator`**: The pose estimator is a drop-in replacement that adds `addVisionMeasurement(pose, timestamp)` — this is how vision integrates cleanly with PathPlanner
- **Configure `AutoBuilder`** inside `__init__`: PathPlanner requires one setup call that registers `getPose`, `resetPose`, and the chassis-speeds drive function

### 3. Fix and Upgrade `VisionLocalizer`

Replace the learning-rate blending approach with:

```python
self.drivetrain.poseEstimator.addVisionMeasurement(visionPose, timestamp)
```

This is the correct way to fuse vision with PathPlanner — it uses the estimator's Kalman filter with proper latency compensation (the `latencyMillisec` field from MegaTag2 is used to timestamp-correct the measurement). This also fixes the missing `adjustOdometry()` method issue.

### 4. Add PathPlanner to the Project

- Add `pathplannerlib` to `pyproject.toml` requires list
- This gives you `AutoBuilder`, `PathPlannerPath`, `PathPlannerAuto`, and `NamedCommands`

### 5. Register Named Commands for Auto Routines

PathPlanner paths can trigger named commands at waypoints. Register things like:

```python
NamedCommands.registerCommand("Shoot", LaunchSequence(self.fuelSubsystem))
NamedCommands.registerCommand("Intake", Intake(self.fuelSubsystem))
```

This is how shooting at specific field positions gets integrated into path following.

### 6. Create Paths in PathPlanner GUI

- Install [PathPlanner](https://github.com/mjansen4857/pathplanner) on your dev machine
- Use the 2026 field image (or a blank field as a mock) to draw paths
- Set path constraints (max speed, max accel) matching your constants
- Drop event markers at positions where you want to shoot
- Export — it generates JSON files that go into `deploy/pathplanner/paths/`

### 7. Update Autonomous Routines in `robotcontainer.py`

Replace `ExampleAuto` with PathPlanner-based autos:

```python
self.autoChooser.addOption("2 Ball Auto", PathPlannerAuto("2BallAuto"))
```

PathPlanner autos are defined in `deploy/pathplanner/autos/` JSON files that reference path files and named commands.

### 8. Drivetrain Characterization (SysId)

PathPlanner's feedforward needs `kS`, `kV`, `kA` to be accurate. Without these, the robot will follow paths sloppily. WPILib's SysId tool can measure these — it requires a few minutes of driving the robot in a straight line in specific patterns. This should be done before competition.

---

## How Vision and PathPlanner Work Together

```
NavX gyro ──────┐
                ├──► DifferentialDrivePoseEstimator ──► PathPlanner AutoBuilder
TalonFX encoders┘         ▲
                           │
Limelight MegaTag2 ────────┘  (addVisionMeasurement with latency compensation)
```

PathPlanner uses the pose estimator's output as ground truth for where the robot is. Limelight continuously corrects encoder drift by pushing AprilTag-based absolute position measurements into the estimator. The result is that the robot follows paths more accurately over longer distances — especially important for multi-step autos that cross the whole field.

---

## Summary Table

| Area                                   | Status      | Work Needed                                                              |
| -------------------------------------- | ----------- | ------------------------------------------------------------------------ |
| `AutoBuilder` configuration            | Missing     | Add in drivetrain `__init__` after all setup                             |
| `pathplannerlib` dependency            | Missing     | Add to `pyproject.toml`                                                  |
| Vision fusion (`addVisionMeasurement`) | Missing     | Rewrite `VisionLocalizer` to call `poseEstimator.addVisionMeasurement()` |
| Named commands for shooting            | Missing     | Register `Shoot`, `Intake` etc. in `RobotContainer`                      |
| PathPlanner path files                 | Missing     | Design in GUI, export to `deploy/pathplanner/`                           |
| Auto routines                          | Basic       | Replace `ExampleAuto` with `PathPlannerAuto`                             |
| Track width constant                   | Placeholder | Measure robot and replace the `0.1` TODO in `CANDriveSubsystem`          |
| Drivetrain characterization (kS/kV/kA) | Missing     | Run SysId before competition                                             |
