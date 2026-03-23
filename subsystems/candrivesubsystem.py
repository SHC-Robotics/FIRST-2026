import commands2

from phoenix6 import hardware, configs, controls, signals
from constants import DriveConstants
from math import pi
from wpimath.estimator import DifferentialDrivePoseEstimator
from wpimath.kinematics import DifferentialDriveKinematics, DifferentialDriveWheelSpeeds, ChassisSpeeds
from wpimath.geometry import Pose2d, Rotation2d
from wpimath.controller import PIDController
from pathplannerlib.auto import AutoBuilder
from pathplannerlib.controller import PPLTVController
from pathplannerlib.config import RobotConfig
import wpilib

import navx


"""
Math is from

https://v6.docs.ctr-electronics.com/en/stable/docs/api-reference/device-specific/talonfx/closed-loop-requests.html#converting-from-meters
"""
WHEEL_RADIUS_METERS = 0.0762  # (3 inches)
GEARBOX_RATIO = 8.45
METERS_PER_ROTATION = (2 * pi * WHEEL_RADIUS_METERS) / GEARBOX_RATIO


class CANDriveSubsystem(commands2.Subsystem):
    def __init__(self) -> None:
        super().__init__()

        # Instantiate motors for drive
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

        # Apply base config to all leaders first
        self.leftLeader.configurator.apply(config, 0.1)
        self.rightLeader.configurator.apply(config, 0.1)

        self.velocity_request = controls.VelocityVoltage(0).with_slot(0)
        self.voltage_request = controls.VoltageOut(0)

        # Left side: clockwise = forward
        config.motor_output.inverted = (
            configs.config_groups.InvertedValue.CLOCKWISE_POSITIVE
        )
        self.leftLeader.configurator.apply(config, 0.1)

        # Right side: counter-clockwise = forward
        config.motor_output.inverted = (
            configs.config_groups.InvertedValue.COUNTER_CLOCKWISE_POSITIVE
        )
        self.rightLeader.configurator.apply(config, 0.1)

        followLeftRequest = controls.Follower(
            DriveConstants.LEFT_LEADER_ID, signals.MotorAlignmentValue.ALIGNED
        )
        self.leftFollower.set_control(followLeftRequest)

        followRightRequest = controls.Follower(
            DriveConstants.RIGHT_LEADER_ID, signals.MotorAlignmentValue.ALIGNED
        )
        self.rightFollower.set_control(followRightRequest)

        # Refresh positions before reading to avoid stale CAN data
        self.leftLeader.get_position().refresh()
        self.rightLeader.get_position().refresh()

        left_position_meters  =  self.leftLeader.get_position().value * METERS_PER_ROTATION
        right_position_meters = -self.rightLeader.get_position().value * METERS_PER_ROTATION

        # NavX calibrates in background — no sleep needed.
        # Check gyro.isCalibrating() in disabledPeriodic() if needed.
        self.gyro = navx.AHRS.create_spi()

        self.kinematics = DifferentialDriveKinematics(
            trackWidth=0.549275  # meters
        )

        self.poseEstimator = DifferentialDrivePoseEstimator(
            self.kinematics,
            self._getGyroRotation(),
            left_position_meters,
            right_position_meters,
            Pose2d(),
        )

        self.field = wpilib.Field2d()
        wpilib.SmartDashboard.putData("Field", self.field)

        config = RobotConfig.fromGUISettings()

        AutoBuilder.configure(
            self.getPose,
            self.resetPose,
            self.getRobotRelativeSpeeds,
            lambda speeds, feedforwards: self.driveRobotRelative(speeds),
            PPLTVController(0.02),
            config,
            shouldFlipPath,
            self
        )

        # kP needs to be tuned. Start small (0.01 - 0.05).
        self.orientationController = PIDController(0.005, 0, 0)
        # Tell the controller that -180 and 180 are the same point so it takes the shortest path
        self.orientationController.enableContinuousInput(-180, 180)
        # Set a tolerance (e.g., stop when within 1 degree of target)
        self.orientationController.setTolerance(1.0)

    # ------------------------------------------------------------------
    # Private helpers
    # ------------------------------------------------------------------

    def _getGyroRotation(self) -> Rotation2d:
        """
        Returns gyro heading as CCW-positive Rotation2d for WPILib compatibility.

        NavX getAngle() is CW-positive and continuously accumulating (no ±180
        boundary jumps), so we negate it for WPILib's CCW-positive convention.

        Use getAngle() rather than getYaw() because getAngle() accumulates
        continuously (e.g. 0 → 720 after two full rotations) whereas getYaw()
        resets at ±180 and would cause pose estimator jumps.
        """
        return Rotation2d.fromDegrees(-self.gyro.getAngle())

    def _getLeftPositionMeters(self) -> float:
        return self.leftLeader.get_position().value * METERS_PER_ROTATION

    def _getRightPositionMeters(self) -> float:
        # Negated because right motor inversion causes get_position() to read
        # negative for forward motion, which contradicts the left encoder.
        return -self.rightLeader.get_position().value * METERS_PER_ROTATION

    # ------------------------------------------------------------------
    # Periodic
    # ------------------------------------------------------------------

    def periodic(self) -> None:
        leftPosition  = self._getLeftPositionMeters()
        rightPosition = self._getRightPositionMeters()

        pose = self.poseEstimator.update(
            self._getGyroRotation(),
            leftPosition,
            rightPosition,
        )

        wpilib.SmartDashboard.putNumber("x", pose.x)
        wpilib.SmartDashboard.putNumber("y", pose.y)
        wpilib.SmartDashboard.putNumber("leftPosition", leftPosition)
        wpilib.SmartDashboard.putNumber("rightPosition", rightPosition)
        wpilib.SmartDashboard.putNumber("heading", pose.rotation().degrees())

        self.field.setRobotPose(pose)

    # ------------------------------------------------------------------
    # Pose
    # ------------------------------------------------------------------

    def getPose(self) -> Pose2d:
        return self.poseEstimator.getEstimatedPosition()

    def resetPose(self, pose: Pose2d) -> None:
        # Do NOT call gyro.reset() here — PathPlanner provides the correct
        # heading baked into pose. Resetting the gyro would zero it and
        # contradict the pose's rotation from the very first loop of auto.
        self.poseEstimator.resetPosition(
            self._getGyroRotation(),
            self._getLeftPositionMeters(),
            self._getRightPositionMeters(),
            pose,
        )

    # ------------------------------------------------------------------
    # Heading
    # ------------------------------------------------------------------

    def getHeading(self) -> Rotation2d:
        return self._getGyroRotation()

    def getRotationSpeed(self) -> float:
        """
        Returns rotational velocity in degrees per second, CCW-positive.

        Negated to match the same CCW-positive convention as _getGyroRotation().
        Used by VisionLocalizer to reject vision updates during fast rotation.
        """
        return -self.gyro.getRate()

    # ------------------------------------------------------------------
    # Speeds
    # ------------------------------------------------------------------

    def getRobotRelativeSpeeds(self) -> ChassisSpeeds:
        return self.kinematics.toChassisSpeeds(self.getWheelSpeeds())

    def getWheelSpeeds(self) -> DifferentialDriveWheelSpeeds:
        left_speed  =  self.leftLeader.get_velocity().value * METERS_PER_ROTATION
        right_speed = -self.rightLeader.get_velocity().value * METERS_PER_ROTATION
        return DifferentialDriveWheelSpeeds(left_speed, right_speed)

    # ------------------------------------------------------------------
    # Drive
    # ------------------------------------------------------------------

    def driveRobotRelative(self, speeds: ChassisSpeeds) -> None:
        wheelSpeeds = self.kinematics.toWheelSpeeds(speeds)
        self.leftLeader.set_control(
            self.velocity_request.with_velocity(wheelSpeeds.left / METERS_PER_ROTATION)
        )
        self.rightLeader.set_control(
            self.velocity_request.with_velocity(wheelSpeeds.right / METERS_PER_ROTATION)
        )

    def driveVolts(self, leftVolts: float, rightVolts: float) -> None:
        # Uses VoltageOut request, not VelocityVoltage which has no with_output()
        self.leftLeader.set_control(self.voltage_request.with_output(leftVolts))
        self.rightLeader.set_control(self.voltage_request.with_output(rightVolts))

    def driveArcade(self, xSpeed: float, zRotation: float) -> None:
        # xSpeed and zRotation are already negated by the Drive command
        # before being passed here, so do not negate again.
        left_percent = xSpeed + zRotation
        right_percent = xSpeed - zRotation

        left_target_rps  = left_percent  * self.MAX_RPS
        right_target_rps = right_percent * self.MAX_RPS

        self.leftLeader.set_control(
            self.velocity_request.with_velocity(left_target_rps)
        )
        self.rightLeader.set_control(
            self.velocity_request.with_velocity(right_target_rps)
        )

    def driveToOrientation(self, targetDegrees: float, xSpeed: float = 0) -> None:
        """
        Rotates the robot toward a target heading while optionally moving forward.
        :param targetDegrees: The target angle in degrees (-180 to 180).
        :param xSpeed: Optional forward speed (0 to 1.0).
        """
        currentDegrees = self.getPose().rotation().degrees()
        rotationOutput = self.orientationController.calculate(currentDegrees, targetDegrees)
        self.driveArcade(xSpeed, rotationOutput)

    def isAtTargetOrientation(self) -> bool:
        return self.orientationController.atSetpoint()


def shouldFlipPath():
    return wpilib.DriverStation.getAlliance() == wpilib.DriverStation.Alliance.kRed
