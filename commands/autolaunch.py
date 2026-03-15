from commands.launchsequence import LaunchSequence
import commands2
from constants import FuelConstants
from subsystems.canfuelsubsystem import CANFuelSubsystem

class AutoLaunch(commands2.SequentialCommandGroup):
    def __init__(self, fuelSubsystem: CANFuelSubsystem) -> None:
        super().__init__()

        self.addCommands(
            commands2.cmd.runOnce(configureLaunchSpeed, fuelSubsystem),
            LaunchSequence(fuelSubsystem, launchTimeout=10)
        )

def configureLaunchSpeed(fuelSubsystem: CANFuelSubsystem):
    fuelSubsystem.multiplier = FuelConstants.AUTO_SHOOTING_MULTIPLIER
