import commands2
import wpilib

from constants import HopperConstants
from subsystems.canhoppersubsystem import CANHopperSubsystem


class GrowHopper(commands2.Command):
    def __init__(self, hopperSubsystem: CANHopperSubsystem) -> None:
        super().__init__()

        self.hopperSubsystem = hopperSubsystem
        self.addRequirements(self.hopperSubsystem)

    def initialize(self) -> None:
        self.hopperSubsystem.setExtension(HopperConstants.EXTENSION_MOTOR_VOLTAGE)

    def end(self, interrupted: bool) -> None:
        self.hopperSubsystem.stop()

    def isFinished(self) -> bool:
        return False

class ShrinkHopper(commands2.Command):
    def __init__(self, hopperSubsystem: CANHopperSubsystem) -> None:
        super().__init__()

        self.hopperSubsystem = hopperSubsystem
        self.addRequirements(self.hopperSubsystem)

    def initialize(self) -> None:
        self.hopperSubsystem.setExtension(-HopperConstants.EXTENSION_MOTOR_VOLTAGE)

    def end(self, interrupted: bool) -> None:
        self.hopperSubsystem.stop()

    def isFinished(self) -> bool:
        return False
