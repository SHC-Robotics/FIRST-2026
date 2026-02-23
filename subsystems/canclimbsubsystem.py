import commands2
import wpilib
import rev

from phoenix6 import hardware, configs, controls
from constants import ClimberConstants

class CANClimbSubsystem(commands2.Subsystem):
    def __init__(self) -> None:
        super().__init__()

        self.leftArm = hardware.TalonFX(ClimberConstants.LEFT_ARM_ID)
        self.rightArm = hardware.TalonFX(ClimberConstants.RIGHT_ARM_ID)

        self.velocity_request = controls.VelocityVoltage(0).with_slot(0)

        config = configs.TalonFXConfiguration()

        config.motor_output.inverted = (
            configs.config_group.InvertedValue.CLOCKWISE_POSITIVE
        )
        self.leftArm.configurator.apply(config)

        config.motor_output.inverted = (
            configs.config_group.InvertedValue.COUNTER_CLOCKWISE_POSITIVE
        )
        self.rightArm.configurator.apply(config)

    def setVoltage(self, voltage: float) -> None:
        self.leftArm.set_control(self.velocity_request.with_voltage(voltage))
        self.rightArm.set_control(self.velocity_request.with_voltage(voltage))

    def stop(self) -> None:
        self.setVoltage(0)
