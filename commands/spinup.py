import commands2
import wpilib

from constants import FuelConstants
from subsystems.canfuelsubsystem import CANFuelSubsystem


class SpinUp(commands2.Command):
    """
    A Command that represents the complete action of spinning up.
    Requires the fuel subsystem.
    """

    def __init__(self, fuelSubsystem: CANFuelSubsystem) -> None:
        super().__init__()

        self.fuelSubsystem = fuelSubsystem
        self.addRequirements(self.fuelSubsystem)

    def initialize(self) -> None:
        self.fuelSubsystem.setIntakeLauncherRoller(
            FuelConstants.LAUNCHING_LAUNCHER_VOLTAGE
        )
        self.fuelSubsystem.setFeederRoller(
            FuelConstants.SPIN_UP_FEEDER_VOLTAGE
        )

    def isFinished(self) -> bool:
        return False
