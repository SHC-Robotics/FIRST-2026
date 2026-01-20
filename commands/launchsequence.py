import commands2

from constants import FuelConstants
from commands.spinup import SpinUp
from commands.launch import Launch
from subsystems.canfuelsubsystem import CANFuelSubsystem


class LaunchSequence(commands2.SequentialCommandGroup):
    """
    A Command that represents the complete action of spinning up for one second
    then launching fuel.
    Requires the fuel subsystem.
    """

    def __init__(self, fuelSubsystem: CANFuelSubsystem) -> None:
        super().__init__()

        self.addCommands(
            SpinUp(fuelSubsystem).withTimeout(FuelConstants.SPIN_UP_SECONDS),
            Launch(fuelSubsystem),
        )
