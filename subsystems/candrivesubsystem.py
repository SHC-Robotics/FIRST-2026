import commands2
import wpilib
import navx

from phoenix6 import CANBus, hardware, configs, controls, signals
from constants import DriveConstants
from time import sleep
from wpimath.kinematics import DifferentialDriveOdometry
from wpimath.geometry import Rotation2d, Pose2d, Translation2d


class CANDriveSubsystem(commands2.Subsystem):
    def __init__(self) -> None:
        super().__init__()

        # Instantiate motors for drive
        self.canivore = CANBus("canivore")
        self.leftLeader = hardware.TalonFX(DriveConstants.LEFT_LEADER_ID)
        self.leftFollower = hardware.TalonFX(DriveConstants.LEFT_FOLLOWER_ID)
        self.rightLeader = hardware.TalonFX(DriveConstants.RIGHT_LEADER_ID)
        self.rightFollower = hardware.TalonFX(DriveConstants.RIGHT_FOLLOWER_ID)

        # Create the configuration to apply to motors. Voltage compensation helps
        # the robot perform more similarly on different battery voltages.
        config = configs.TalonFXConfiguration()

        slot0 = config.slot0
        slot0.k_p = 0.3
        self.MAX_RPS = 70
        self.leftLeader.configurator.apply(config)
        self.rightLeader.configurator.apply(config)

        self.velocity_request = controls.VelocityVoltage(0)

        config.motor_output.neutral_mode = signals.NeutralModeValue.BRAKE
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

        self.leftOut = controls.DutyCycleOut(0)
        self.rightOut = controls.DutyCycleOut(0)

        self.leftEncoder = hardware.CANcoder(
            DriveConstants.LEFT_LEADER_ID, self.canivore
        )
        self.rightEncoder = hardware.CANcoder(
            DriveConstants.RIGHT_LEADER_ID, self.canivore
        )

        self.gyro = navx.AHRS.create_spi()
        sleep(1.0)

        self.odometry = DifferentialDriveOdometry(
            self.gyro.getRotation2d(),
            self.leftEncoder.get_position().value * 1,
            self.rightEncoder.get_position().value * -1,
        )

        self.odometryHeadingOffset = Rotation2d()
        self.resetOdometry(Pose2d(0, 0, 0))
        self.field = wpilib.Field2d()

    def resetOdometry(self, pose):
        self.gyro.reset()
        self.odometry.resetPosition(
            self.gyro.getRotation2d(),
            self.leftEncoder.get_position().value * 1,
            self.rightEncoder.get_position().value * -1,
            pose,
        )
        self.odometryHeadingOffset = (
            self.odometry.getPose().rotation() - self.getGyroHeading()
        )

    def getPose(self):
        return self.odometry.getPose()

    def getGyroHeading(self):
        return self.gyro.getRotation2d()

    def periodic(self):
        pose = self.odometry.update(
            self.gyro.getRotation2d(),
            self.leftEncoder.get_position().value * 1,
            self.rightEncoder.get_position().value * -1,
        )

        wpilib.SmartDashboard.putNumber("x", pose.x)
        wpilib.SmartDashboard.putNumber("y", pose.y)
        wpilib.SmartDashboard.putNumber("heading", pose.rotation().degrees())
        self.field.setRobotPose(pose)

    def driveArcade(self, xSpeed: float, zRotation: float) -> None:
        xSpeed = -xSpeed

        left_percent = xSpeed + zRotation
        right_percent = xSpeed - zRotation

        max_input = max(abs(left_percent), abs(right_percent), 1.0)
        left_normalized = left_percent / max_input
        right_normalized = right_percent / max_input

        left_target_rps = left_normalized * self.MAX_RPS
        right_target_rps = right_normalized * self.MAX_RPS

        #self.leftLeader.set_control(self.leftOut.with_output(xSpeed + zRotation))
        #self.rightLeader.set_control(self.rightOut.with_output(xSpeed - zRotation))

        self.leftLeader.set_control(self.velocity_request.with_velocity(left_target_rps))
        self.rightLeader.set_control(self.velocity_request.with_velocity(right_target_rps))
