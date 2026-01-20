import commands2
import wpilib

from constants import FuelConstants
from subsystems.canfuelsubsystem import CANFuelSubsystem


class SpinUp(commands2.Command):
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
            wpilib.SmartDashboard.getNumber(
                "Spin-up feeder roller value", FuelConstants.SPIN_UP_FEEDER_VOLTAGE
            )
        )

    def isFinished(self) -> bool:
        return False
