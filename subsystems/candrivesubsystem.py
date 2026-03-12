import commands2
from commands2 import sysid
 
from phoenix6 import hardware, configs, controls, signals
from phoenix6.signals import SignalLogger
from constants import DriveConstants
from math import pi
from wpimath.estimator import DifferentialDrivePoseEstimator
from wpimath.kinematics import DifferentialDriveKinematics, DifferentialDriveWheelSpeeds, ChassisSpeeds
from wpimath.geometry import Pose2d, Rotation2d
from wpimath.controller import PIDController
from wpilib.sysid import SysIdRoutineLog
from pathplannerlib.auto import AutoBuilder
from pathplannerlib.controller import PPLTVController
from pathplannerlib.config import RobotConfig
import wpilib
 
import navx
from time import sleep
 
 
"""
Math is from
https://v6.docs.ctr-electronics.com/en/stable/docs/api-reference/device-specific/talonfx/closed-loop-requests.html#converting-from-meters
"""
WHEEL_RADIUS_METERS = 0.0762  # 3 inches
GEARBOX_RATIO = 8.45
METERS_PER_ROTATION = (2 * pi * WHEEL_RADIUS_METERS) / GEARBOX_RATIO
 
 
class CANDriveSubsystem(commands2.Subsystem):
    def __init__(self) -> None:
        super().__init__()
 
        # --- Motors ---
        self.leftLeader = hardware.TalonFX(DriveConstants.LEFT_LEADER_ID)
        self.leftFollower = hardware.TalonFX(DriveConstants.LEFT_FOLLOWER_ID)
        self.rightLeader = hardware.TalonFX(DriveConstants.RIGHT_LEADER_ID)
        self.rightFollower = hardware.TalonFX(DriveConstants.RIGHT_FOLLOWER_ID)
 
        # Create the configuration to apply to motors. Voltage compensation helps
        # the robot perform more similarly on different battery voltages.
        config = configs.TalonFXConfiguration()
 
        slot0 = config.slot0
        slot0.k_p = 0.1
        slot0.k_v = 0.12
        slot0.k_s = 0.1
        slot0.k_i = 0
        slot0.k_d = 0
 
        self.MAX_RPS = 70
        self.leftLeader.configurator.apply(config)
        self.rightLeader.configurator.apply(config)
 
        # TODO: braking
        # config.motor_output.neutral_mode = signals.NeutralModeValue.BRAKE
        config.motor_output.inverted = (
            configs.config_groups.InvertedValue.CLOCKWISE_POSITIVE
        )
        self.leftLeader.configurator.apply(config)
 
        config.motor_output.inverted = (
            configs.config_groups.InvertedValue.COUNTER_CLOCKWISE_POSITIVE
        )
        self.rightLeader.configurator.apply(config)
 
        # --- Followers ---
        self.leftFollower.set_control(
            controls.Follower(DriveConstants.LEFT_LEADER_ID, signals.MotorAlignmentValue.ALIGNED)
        )
        self.rightFollower.set_control(
            controls.Follower(DriveConstants.RIGHT_LEADER_ID, signals.MotorAlignmentValue.ALIGNED)
        )
 
        # --- Control requests ---
        self.velocity_request = controls.VelocityVoltage(0).with_slot(0)
        self.voltage_request = controls.VoltageOut(0)
 
        # --- Gyro ---
        self.gyro = navx.AHRS.create_spi()
        sleep(1.0)
 
        # --- Kinematics & Odometry ---
        self.kinematics = DifferentialDriveKinematics(trackWidth=0.549275)
 
        left_position_meters = self.leftLeader.get_position().value * METERS_PER_ROTATION
        right_position_meters = self.rightLeader.get_position().value * METERS_PER_ROTATION
 
        self.poseEstimator = DifferentialDrivePoseEstimator(
            self.kinematics,
            self.gyro.getRotation2d(),
            left_position_meters,
            right_position_meters,
            Pose2d(),
        )
 
        self.field = wpilib.Field2d()
        wpilib.SmartDashboard.putData("Field", self.field)
 
        # --- PathPlanner ---
        robot_config = RobotConfig.fromGUISettings()
        AutoBuilder.configure(
            self.getPose,
            self.resetPose,
            self.getRobotRelativeSpeeds,
            lambda speeds, feedforwards: self.driveRobotRelative(speeds),
            PPLTVController(0.02),
            robot_config,
            shouldFlipPath,
            self,
        )
 
        # --- Orientation PID ---
        self.orientationController = PIDController(0.005, 0, 0)
        self.orientationController.enableContinuousInput(-180, 180)
        self.orientationController.setTolerance(1.0)
 
        # --- SysId Routine ---
        # Uses SignalLogger (Phoenix 6) for high-frequency Talon data.
        # Press Back on controller to start logging, Start to stop.
        self.sysIdRoutine = sysid.SysIdRoutine(
            sysid.SysIdRoutine.Config(
                rampRate=1.0,       # V/s for quasistatic test
                stepVoltage=7.0,    # V for dynamic test
                timeout=10.0,       # Max seconds per test direction
                recordState=lambda state: SignalLogger.write_string(
                    "sysid-test-state", state.toString()
                ),
            ),
            sysid.SysIdRoutine.Mechanism(
                self.sysIdDrive,
                self.sysIdLog,
                self,
            ),
        )
 
    # ------------------------------------------------------------------ #
    #  Periodic                                                            #
    # ------------------------------------------------------------------ #
 
    def periodic(self) -> None:
        pose = self.poseEstimator.update(
            self.gyro.getRotation2d(),
            self.leftLeader.get_position().value * METERS_PER_ROTATION,
            self.rightLeader.get_position().value * METERS_PER_ROTATION,
        )
        wpilib.SmartDashboard.putNumber("x", pose.x)
        wpilib.SmartDashboard.putNumber("y", pose.y)
        wpilib.SmartDashboard.putNumber("heading", pose.rotation().degrees())
        self.field.setRobotPose(pose)
 
    # ------------------------------------------------------------------ #
    #  Pose / Odometry                                                     #
    # ------------------------------------------------------------------ #
 
    def getPose(self) -> Pose2d:
        return self.poseEstimator.getEstimatedPosition()
 
    def resetPose(self, pose: Pose2d) -> None:
        self.poseEstimator.resetPosition(
            self.gyro.getRotation2d(),
            self.leftLeader.get_position().value * METERS_PER_ROTATION,
            self.rightLeader.get_position().value * METERS_PER_ROTATION,
            pose,
        )
 
    def getHeading(self) -> Rotation2d:
        return self.gyro.getRotation2d()
 
    def getWheelSpeeds(self) -> DifferentialDriveWheelSpeeds:
        left_speed = self.leftLeader.get_velocity().value * METERS_PER_ROTATION
        right_speed = self.rightLeader.get_velocity().value * METERS_PER_ROTATION
        return DifferentialDriveWheelSpeeds(left_speed, right_speed)
 
    def getRobotRelativeSpeeds(self) -> ChassisSpeeds:
        return self.kinematics.toChassisSpeeds(self.getWheelSpeeds())
 
    # ------------------------------------------------------------------ #
    #  Drive methods                                                       #
    # ------------------------------------------------------------------ #
 
    def driveRobotRelative(self, speeds: ChassisSpeeds) -> None:
        wheelSpeeds = self.kinematics.toWheelSpeeds(speeds)
        self.leftLeader.set_control(
            self.velocity_request.with_velocity(wheelSpeeds.left / METERS_PER_ROTATION)
        )
        self.rightLeader.set_control(
            self.velocity_request.with_velocity(wheelSpeeds.right / METERS_PER_ROTATION)
        )
 
    def driveVolts(self, leftVolts: float, rightVolts: float) -> None:
        self.leftLeader.set_control(self.voltage_request.with_output(leftVolts))
        self.rightLeader.set_control(self.voltage_request.with_output(rightVolts))
 
    def driveArcade(self, xSpeed: float, zRotation: float) -> None:
        xSpeed = -xSpeed
        left_target_rps = (xSpeed + zRotation) * self.MAX_RPS
        right_target_rps = (xSpeed - zRotation) * self.MAX_RPS
        self.leftLeader.set_control(self.velocity_request.with_velocity(left_target_rps))
        self.rightLeader.set_control(self.velocity_request.with_velocity(right_target_rps))
 
    def driveToOrientation(self, targetDegrees: float, xSpeed: float = 0) -> None:
        """
        Rotates the robot toward a target heading while optionally moving forward.
        :param targetDegrees: Target angle in degrees (-180 to 180).
        :param xSpeed: Optional forward speed (0 to 1.0).
        """
        currentDegrees = self.getPose().rotation().degrees()
        rotationOutput = self.orientationController.calculate(currentDegrees, targetDegrees)
        self.driveArcade(xSpeed, rotationOutput)
 
    def isAtTargetOrientation(self) -> bool:
        return self.orientationController.atSetpoint()
 
    # ------------------------------------------------------------------ #
    #  SysId                                                               #
    # ------------------------------------------------------------------ #
 
    def sysIdDrive(self, voltage: float) -> None:
        """Drive both sides at the same voltage during characterization."""
        self.driveVolts(voltage, voltage)
 
    def sysIdLog(self, log: SysIdRoutineLog) -> None:
        """
        Log linear position (m) and velocity (m/s) for SysId analysis.
        SignalLogger captures high-frequency Talon data automatically;
        this WPILog callback provides a backup and is required by the API.
        """
        (log.motor("drive-left")
            .voltage(self.leftLeader.get_motor_voltage().value)
            .linearPosition(self.leftLeader.get_position().value * METERS_PER_ROTATION)
            .linearVelocity(self.leftLeader.get_velocity().value * METERS_PER_ROTATION))
 
        (log.motor("drive-right")
            .voltage(self.rightLeader.get_motor_voltage().value)
            .linearPosition(self.rightLeader.get_position().value * METERS_PER_ROTATION)
            .linearVelocity(self.rightLeader.get_velocity().value * METERS_PER_ROTATION))
 
    def sysIdQuasistatic(self, direction: sysid.SysIdRoutine.Direction) -> commands2.Command:
        """Slowly ramps voltage — characterizes kS and kV (max velocity)."""
        return self.sysIdRoutine.quasistatic(direction)
 
    def sysIdDynamic(self, direction: sysid.SysIdRoutine.Direction) -> commands2.Command:
        """Steps voltage — characterizes kA (moment of inertia)."""
        return self.sysIdRoutine.dynamic(direction)
 
 
def shouldFlipPath() -> bool:
    return wpilib.DriverStation.getAlliance() == wpilib.DriverStation.Alliance.kRed

