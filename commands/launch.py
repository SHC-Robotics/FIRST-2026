import commands2
import wpilib

from constants import FuelConstants, AprilTagIds
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
        wpilib.SmartDashboard.putString("Current command", "Launch")
        print("Current command: launch")

        multiplier = self.fuelSubsystem.multiplier

        self.fuelSubsystem.setIntakeLauncherRoller(
            FuelConstants.LAUNCHING_LAUNCHER_VOLTAGE * multiplier
        )
        self.fuelSubsystem.setFeederRoller(
            FuelConstants.LAUNCHING_FEEDER_VOLTAGE
        )
    
    def isFinished(self) -> bool:
        return False

    def end(self, interrupted: bool) -> None:
        self.fuelSubsystem.stop()

class AutoLaunch(commands2.Command):
    """
    A Command that represents the complete action of launching fuel.
    Requires the fuel subsystem.
    """

    def __init__(self, fuelSubsystem: CANFuelSubsystem) -> None:
        super().__init__()

        self.fuelSubsystem = fuelSubsystem
        self.addRequirements(self.fuelSubsystem)

    def initialize(self) -> None:
        wpilib.SmartDashboard.putString("Current command", "Launch")
        print("Current command: launch")

        multiplier = self.fuelSubsystem.multiplier

        self.fuelSubsystem.setIntakeLauncherRoller(
            FuelConstants.LAUNCHING_LAUNCHER_VOLTAGE * multiplier
        )
        self.fuelSubsystem.setFeederRoller(
            FuelConstants.AUTO_LAUNCHING_FEEDER_VOLTAGE)
    def isFinished(self) -> bool:
        return False

    def end(self, interrupted: bool) -> None:
        self.fuelSubsystem.stop()
    

class StopLaunch(commands2.Command):
    def __init__(self, fuelSubsystem: CANFuelSubsystem) -> None:
        super().__init__()

        self.fuelSubsystem = fuelSubsystem
        self.addRequirements(self.fuelSubsystem)

    def initialize(self) -> None:
        self.fuelSubsystem.stop()
