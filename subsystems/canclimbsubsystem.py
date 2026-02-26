import commands2
import wpilib
import rev

from phoenix6 import hardware, configs, controls, signals
from constants import ClimberConstants

class CANClimbSubsystem(commands2.Subsystem):
    def __init__(self) -> None:
        super().__init__()

        self.leftArm = hardware.TalonFX(ClimberConstants.LEFT_ARM_ID)
        #.rightArm = hardware.TalonFX(ClimberConstants.RIGHT_ARM_ID)

        # self.velocity_request = controls.VelocityVoltage(0).with_slot(0)

        self.duty_cycle_request = controls.DutyCycleOut(0)

        config = configs.TalonFXConfiguration()

        #config.motor_output.neutral_mode = signals.NeutralModeValue.BRAKE

        config.motor_output.inverted = (
            configs.config_groups.InvertedValue.CLOCKWISE_POSITIVE
        )
        self.leftArm.configurator.apply(config)

        config.motor_output.inverted = (
            configs.config_groups.InvertedValue.COUNTER_CLOCKWISE_POSITIVE
        )


        #self.rightArm.configurator.apply(config)

    def setVoltage(self, voltage: float) -> None:
        self.leftArm.set_control(self.duty_cycle_request.with_output(voltage))
        # self.leftArm.set_control(self.velocity_request.with_velocity(voltage))
        #self.rightArm.set_control(self.velocity_request.with_voltage(voltage))

    def stop(self) -> None:
        self.setVoltage(0)
