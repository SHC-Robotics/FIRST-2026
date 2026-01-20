import commands2

from constants import OperatorConstants
from subsystems.candrivesubsystem import CANDriveSubsystem

class Drive(commands2.Command):
    def __init__(self, driveSubsystem: CANDriveSubsystem, driverController) -> None:
        super().__init__()
        self.driveSubsystem = driveSubsystem
        self.controller = driverController
        self.addRequirements(self.driveSubsystem)

    def execute(self) -> None:
        self.driveSubsystem.driveArcade(
            -self.controller.getLeftY() * OperatorConstants.DRIVE_SCALING,
            -self.controller.getRightX() * OperatorConstants.ROTATION_SCALING,
        )

    def end(self, interrupted: bool) -> None:
        self.driveSubsystem.driveArcade(0, 0)

    def isFinished(self) -> bool:
        return False
