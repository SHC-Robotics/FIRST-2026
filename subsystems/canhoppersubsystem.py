import commands2
import math
from constants import HopperConstants
import rev
import wpilib

class CANHopperSubsystem(commands2.Subsystem):
    def __init__(self, operatorController) -> None:
        super().__init__()

        self.controller = operatorController

        self.extensionMotor = rev.SparkMax(
            HopperConstants.EXTENSION_MOTOR_ID,
            rev.SparkLowLevel.MotorType.kBrushless,
        )

        extensionConfig = rev.SparkMaxConfig()
        extensionConfig.smartCurrentLimit(HopperConstants.EXTENSION_MOTOR_CURRENT_LIMIT)
        extensionConfig.setIdleMode(rev.SparkBaseConfig.IdleMode.kBrake)
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

    def periodic(self):
        # motorExtension = 0
        # leftY = -self.controller.getLeftY()
        # if self.getExtensionPosition() < HopperConstants.EXTENSION_NUM_ROTATIONS and leftY > 0.2:
        #     motorExtension = HopperConstants.EXTENSION_MOTOR_VOLTAGE
        # if self.getExtensionPosition() > 0.1 and leftY < -0.2:
        #     motorExtension = -HopperConstants.EXTENSION_MOTOR_VOLTAGE
        # self.setExtension(motorExtension)
        
        # wpilib.SmartDashboard.putNumber("Hopper position", self.getExtensionPosition())
        pass