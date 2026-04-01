from commands.launchsequence import LaunchSequence
import commands2
from constants import FuelConstants
from subsystems.candrivesubsystem import CANDriveSubsystem
from subsystems.canfuelsubsystem import CANFuelSubsystem

class AutoLaunch(commands2.SequentialCommandGroup):
    def __init__(self, fuelSubsystem: CANFuelSubsystem) -> None:
        super().__init__()

        self.addCommands(
            commands2.cmd.runOnce(lambda: configureLaunchSpeed(fuelSubsystem), fuelSubsystem),
            LaunchSequence(fuelSubsystem, launchTimeout=15)
        )

class AutoDrive(commands2.Command):
    def __init__(
            self, driveSubsystem: CANDriveSubsystem, xSpeed: float, zRotation: float
    ) -> None:
        super().__init__()

        self.driveSubsystem = driveSubsystem
        self.xSpeed = xSpeed
        self.zRotation = zRotation
        self.addRequirements(self.driveSubsystem)

    def execute(self) -> None:
        self.driveSubsystem.driveArcade(-self.xSpeed, self.zRotation)

    def end(self, interrupted: bool) -> None:
        self.driveSubsystem.driveArcade(0, 0)

    def isFinished(self) -> bool:
        return False

class AutoBackUpLaunch(commands2.SequentialCommandGroup):
    def __init__(self, driveSubsystem: CANDriveSubsystem, fuelSubsystem: CANFuelSubsystem) -> None:
        super().__init__()

        self.addCommands(
            AutoDrive(driveSubsystem, -0.1, 0.1).withTimeout(3),
            commands2.cmd.runOnce(lambda: configureLaunchSpeed(fuelSubsystem), fuelSubsystem),
            LaunchSequence(fuelSubsystem, launchTimeout=15)
        )

def configureLaunchSpeed(fuelSubsystem: CANFuelSubsystem):
    fuelSubsystem.multiplier = FuelConstants.AUTO_SHOOTING_MULTIPLIER
