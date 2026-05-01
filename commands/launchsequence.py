import commands2

from constants import FuelConstants
from commands.spinup import SpinUp
from commands.launch import Launch, AutoLaunch
from subsystems.canfuelsubsystem import CANFuelSubsystem


class LaunchSequence(commands2.SequentialCommandGroup):
    """
    A Command that represents the complete action of spinning up for one second
    then launching fuel.
    Requires the fuel subsystem.
    """

    def __init__(self, fuelSubsystem: CANFuelSubsystem, launchTimeout=None, auto=False) -> None:
        super().__init__()

        if auto:
            if launchTimeout:
                self.addCommands(
                    SpinUp(fuelSubsystem).withTimeout(FuelConstants.AUTO_SPIN_UP_SECONDS),
                    AutoLaunch(fuelSubsystem).withTimeout(launchTimeout),
                )
            else: 
                self.addCommands(
                    SpinUp(fuelSubsystem).withTimeout(FuelConstants.AUTO_SPIN_UP_SECONDS),
                    AutoLaunch(fuelSubsystem),
                )
        else:
            if launchTimeout:
                self.addCommands(
                    SpinUp(fuelSubsystem).withTimeout(FuelConstants.SPIN_UP_SECONDS),
                    Launch(fuelSubsystem).withTimeout(launchTimeout),
                )
            else: 
                self.addCommands(
                    SpinUp(fuelSubsystem).withTimeout(FuelConstants.SPIN_UP_SECONDS),
                    Launch(fuelSubsystem),
                )
        
