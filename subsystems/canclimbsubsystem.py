import commands2
import wpilib

from phoenix6 import hardware, configs, controls, signals
from constants import ClimberConstants

class CANClimbSubsystem(commands2.Subsystem):
    def __init__(self) -> None:
        super().__init__()

        self.leftArm = hardware.TalonFX(ClimberConstants.LEFT_ARM_ID)
        # self.rightArm = hardware.TalonFX(ClimberConstants.RIGHT_ARM_ID)

        self.duty_cycle_request = controls.DutyCycleOut(0)

        config = configs.TalonFXConfiguration()

        config.motor_output.neutral_mode = signals.NeutralModeValue.BRAKE

        config.motor_output.inverted = (
            configs.config_groups.InvertedValue.CLOCKWISE_POSITIVE
        )
        self.leftArm.configurator.apply(config)

        # config.motor_output.inverted = (
        #     configs.config_groups.InvertedValue.COUNTER_CLOCKWISE_POSITIVE
        # )
        # self.rightArm.configurator.apply(config)

        self.initLeftPosition = self.leftArm.get_position().value
        # self.initRightPosition = self.rightArm.get_position().value

    def setVoltage(self, voltage: float) -> None:
        self.leftArm.set_control(self.duty_cycle_request.with_output(voltage))
        # self.rightArm.set_control(self.duty_cycle_request.with_output(voltage))

    def stop(self) -> None:
        self.setVoltage(0)
