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
        extensionConfig.setIdleMode(rev.SparkBaseConfig.IdleMode.kCoast)
        # extensionConfig.inverted(True)
        self.extensionMotor.configure(
            extensionConfig,
            rev.ResetMode.kResetSafeParameters,
            rev.PersistMode.kPersistParameters,
        )

        self.extensionEncoder = self.extensionMotor.getEncoder()

        self.setExtensionPosition(0)

    def setExtension(self, voltage: float) -> None:
        """Set voltage of extension motor, where positive direction is growing the hopper."""
        self.extensionMotor.setVoltage(voltage)

    def getExtensionPosition(self) -> float:
        """Get the relative encoder position of the extension motor, in rotations."""
        return self.extensionEncoder.getPosition()

    def setExtensionPosition(self, rotations) -> None:
        """Set the relative encoder position of the extension motor, in rotations."""
        return self.extensionEncoder.setPosition(rotations)

    def stop(self) -> None:
        self.extensionMotor.set(0)

