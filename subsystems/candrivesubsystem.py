import commands2

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

        slot0 = config.slot0
        slot0.k_p = 0.1
        slot0.k_v = 0.12
        slot0.k_s = 0.1
        slot0.k_i = 0
        slot0.k_d = 0
        self.MAX_RPS = 11
        self.leftLeader.configurator.apply(config)
        self.rightLeader.configurator.apply(config)

        self.velocity_request = controls.VelocityVoltage(0).with_slot(0)

        #config.motor_output.neutral_mode = signals.NeutralModeValue.BRAKE
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

    def driveArcade(self, xSpeed: float, zRotation: float) -> None:
        xSpeed = -xSpeed

        left_percent = xSpeed + zRotation
        right_percent = xSpeed - zRotation

        left_target_rps = left_percent * self.MAX_RPS
        right_target_rps = right_percent * self.MAX_RPS

        print(self.leftLeader.get_velocity().value)

        self.leftLeader.set_control(
            self.velocity_request.with_velocity(left_target_rps)
        )
        self.rightLeader.set_control(
            self.velocity_request.with_velocity(right_target_rps)
        )
