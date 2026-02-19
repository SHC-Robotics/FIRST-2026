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

from time import sleep


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
        self.leftLeader.configurator.apply(config)
        self.rightLeader.configurator.apply(config)

        self.velocity_request = controls.VelocityVoltage(0).with_slot(0)

        # config.motor_output.neutral_mode = signals.NeutralModeValue.BRAKE
        config.motor_output.inverted = (
            configs.config_groups.InvertedValue.CLOCKWISE_POSITIVE
        )
        self.leftLeader.configurator.apply(config)

        config.motor_output.inverted = (
            configs.config_groups.InvertedValue.COUNTER_CLOCKWISE_POSITIVE
        )
        self.rightLeader.configurator.apply(config)

        followLeftRequest = controls.Follower(
            DriveConstants.LEFT_LEADER_ID, signals.MotorAlignmentValue.ALIGNED
        )
        self.leftFollower.set_control(followLeftRequest)

        followRightRequest = controls.Follower(
            DriveConstants.RIGHT_LEADER_ID, signals.MotorAlignmentValue.ALIGNED
        )
        self.rightFollower.set_control(followRightRequest)

        ## Easier way to read encoder than using cancoder.
        left_position_meters = (
            self.leftLeader.get_position().value * METERS_PER_ROTATION
        )
        right_position_meters = (
            self.rightLeader.get_position().value * METERS_PER_ROTATION
        )

        self.gyro = navx.AHRS.create_spi()
        sleep(1.0)

        self.kinematics = DifferentialDriveKinematics(
            trackWidth=0.1  # TODO: measure track width and update this value (meters)
        )

        ## Supposed to be a drop in replacement from DifferentialDriveOdometry
        # Meant to be easier to use with loczlization
        self.poseEstimator = DifferentialDrivePoseEstimator(
            self.kinematics,
            self.gyro.getRotation2d(),  # initial heading
            left_position_meters,  # initial left encoder distance
            right_position_meters,  # initial right encoder distance
            Pose2d(),  # starting pose. Does not matter because, it will be reset by PathPlanner using resetOdometry()
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

        #WPIlib PID setup for rotating towards an angle
        #unsure if WPILIB pid or talon PID better to use here
        #need to do vast research on how this works

        # kP needs to be tuned. Start small (0.01 - 0.05).
        self.orientationController = PIDController(0.005, 0, 0) 
        # Tell the controller that -180 and 180 are the same point so it takes the shortest path
        self.orientationController.enableContinuousInput(-180, 180)
        # Set a tolerance (e.g., stop when within 1 degree of target)
        self.orientationController.setTolerance(1.0)




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

    def getPose(self) -> Pose2d:
        return self.poseEstimator.getEstimatedPosition()

    def resetPose(self, pose: Pose2d) -> None:
        # self.gyro.reset()
        self.poseEstimator.resetPosition(
            self.gyro.getRotation2d(),
            self.leftLeader.get_position().value * METERS_PER_ROTATION,
            self.rightLeader.get_position().value * METERS_PER_ROTATION,
            pose,
        )

    def getRobotRelativeSpeeds(self) -> ChassisSpeeds:
        return self.kinematics.toChassisSpeeds(self.getWheelSpeeds())

    def driveRobotRelative(self, speeds: ChassisSpeeds):
        # speeds = speeds.discretize(speeds, 0.02)
        wheelSpeeds = self.kinematics.toWheelSpeeds(speeds)
        self.leftLeader.set_control(self.velocity_request.with_velocity(wheelSpeeds.left / METERS_PER_ROTATION))
        self.rightLeader.set_control(self.velocity_request.with_velocity(wheelSpeeds.right / METERS_PER_ROTATION))

    def getHeading(self) -> Rotation2d:
        return self.gyro.getRotation2d()

    def getWheelSpeeds(self) -> DifferentialDriveWheelSpeeds:
        left_speed = self.leftLeader.get_velocity().value * METERS_PER_ROTATION
        right_speed = self.rightLeader.get_velocity().value * METERS_PER_ROTATION
        return DifferentialDriveWheelSpeeds(left_speed, right_speed)

    def driveVolts(self, leftVolts: float, rightVolts: float) -> None:
        self.leftLeader.set_control(self.velocity_request.with_voltage(leftVolts))
        self.rightLeader.set_control(self.velocity_request.with_voltage(rightVolts))

    def driveArcade(self, xSpeed: float, zRotation: float) -> None:
        xSpeed = -xSpeed

        left_percent = xSpeed + zRotation
        right_percent = xSpeed - zRotation

        left_target_rps = left_percent * self.MAX_RPS
        right_target_rps = right_percent * self.MAX_RPS

        self.leftLeader.set_control(
            self.velocity_request.with_velocity(left_target_rps)
        )
        self.rightLeader.set_control(
            self.velocity_request.with_velocity(right_target_rps)
        )

    def driveToOrientation(self, targetDegrees: float, xSpeed: float = 0):
        """
        Rotates the robot toward a target heading while optionally moving forward.
        :param targetDegrees: The target angle in degrees (-180 to 180).
        :param xSpeed: Optional forward speed (0 to 1.0).
        """
        currentDegrees = self.getPose().rotation().degrees()
        print("Current Degrees" + str(currentDegrees))
        print("Target: " + str(targetDegrees))
        
        # Calculate the rotation output needed
        # This returns a value typically between -1.0 and 1.0
        rotationOutput = self.orientationController.calculate(currentDegrees, targetDegrees)
        
        # Use your existing arcade drive to move the motors
        self.driveArcade(xSpeed, rotationOutput)

    def isAtTargetOrientation(self) -> bool:
        return self.orientationController.atSetpoint()

    

def shouldFlipPath():
    return wpilib.DriverStation.getAlliance() == wpilib.DriverStation.Alliance.kRed
