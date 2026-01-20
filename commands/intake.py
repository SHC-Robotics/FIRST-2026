import commands2
import wpilib

from constants import FuelConstants
from subsystems.canfuelsubsystem import CANFuelSubsystem


class Intake(commands2.Command):
    def __init__(self, fuelSubsystem: CANFuelSubsystem) -> None:
        super().__init__()
        self.fuelSubsystem = fuelSubsystem
        self.addRequirements(self.fuelSubsystem)

    def initialize(self) -> None:
        self.fuelSubsystem.setIntakeLauncherRoller(
            wpilib.SmartDashboard.getNumber(
                "Intaking intake roller value", FuelConstants.INTAKING_INTAKE_VOLTAGE
            )
        )
        self.fuelSubsystem.setFeederRoller(
            wpilib.SmartDashboard.getNumber(
                "Intaking feeder roller value", FuelConstants.INTAKING_FEEDER_VOLTAGE
            )
        )

    def end(self, interrupted: bool) -> None:
        self.fuelSubsystem.setIntakeLauncherRoller(0)
        self.fuelSubsystem.setFeederRoller(0)

    def isFinished(self) -> bool:
        return False
