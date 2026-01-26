import commands2
from wpilib.drive import DifferentialDrive

from phoenix6 import CANBus, hardware, configs, controls, signals
from constants import DriveConstants


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
        config.motor_output.inverted = (
            configs.config_groups.InvertedValue.COUNTER_CLOCKWISE_POSITIVE
        )
        self.leftLeader.configurator.apply(config)

        config.motor_output.inverted = (
            configs.config_groups.InvertedValue.CLOCKWISE_POSITIVE
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

        # Instantiate differential drive class
        self.drive = DifferentialDrive(self.leftLeader, self.rightLeader)
        self.drive.setMaxOutput(0.5)

    def driveArcade(self, xSpeed: float, zRotation: float) -> None:
        self.drive.arcadeDrive(xSpeed, zRotation)
