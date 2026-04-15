import commands2
from constants import HopperConstants
import rev
import wpilib

class CANHopperSubsystem(commands2.Subsystem):
    def __init__(self) -> None:
        super().__init__()

        self.extensionMotor = rev.SparkMax(
            HopperConstants.EXTENSION_MOTOR_ID,
            rev.SparkLowLevel.MotorType.kBrushless,
        )

        extensionConfig = rev.SparkMaxConfig()
        extensionConfig.smartCurrentLimit(HopperConstants.EXTENSION_MOTOR_CURRENT_LIMIT)
        # extensionConfig.inverted(True)
        self.extensionMotor.configure(
            extensionConfig,
            rev.ResetMode.kResetSafeParameters,
            rev.PersistMode.kPersistParameters,
        )

    def setExtension(self, voltage: float) -> None:
        self.extensionMotor.setVoltage(voltage)

    def stop(self) -> None:
        self.extensionMotor.set(0)

