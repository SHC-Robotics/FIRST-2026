import commands2
import wpilib

from constants import FuelConstants
from subsystems.canfuelsubsystem import CANFuelSubsystem


class Launch(commands2.Command):
    """
    A Command that represents the complete action of launching fuel.
    Requires the fuel subsystem.
    """

    def __init__(self, fuelSubsystem: CANFuelSubsystem) -> None:
        super().__init__()

        self.fuelSubsystem = fuelSubsystem
        self.addRequirements(self.fuelSubsystem)

    def initialize(self) -> None:
        self.fuelSubsystem.setIntakeLauncherRoller(
            wpilib.SmartDashboard.getNumber(
                "Launching launcher roller value",
                FuelConstants.LAUNCHING_LAUNCHER_VOLTAGE,
            )
        )
        self.fuelSubsystem.setFeederRoller(
            -1
            * wpilib.SmartDashboard.getNumber(
                "Launching feeder roller value", FuelConstants.LAUNCHING_FEEDER_VOLTAGE
            )
        )

    def isFinished(self) -> bool:
        return False
