import commands2

from commands.autodrive import AutoDrive
from commands.launch import Launch
from subsystems.candrivesubsystem import CANDriveSubsystem
from subsystems.canfuelsubsystem import CANFuelSubsystem


class ExampleAuto(commands2.SequentialCommandGroup):
    """
    A Command sequence that represents driving for 0.25 seconds then launching fuel for 10 seconds.
    Requires the drive subsystem and fuel subsystem.
    """

    def __init__(
        self, driveSubsystem: CANDriveSubsystem, fuelSubsystem: CANFuelSubsystem
    ) -> None:
        super().__init__()
        self.addCommands(
            AutoDrive(driveSubsystem, 0.5, 0.0).withTimeout(0.25),
            Launch(fuelSubsystem).withTimeout(10),
        )
