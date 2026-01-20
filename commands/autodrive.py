import commands2

from subsystems.candrivesubsystem import CANDriveSubsystem


class AutoDrive(commands2.Command):
    """
    A Command that represents the complete action of driving given a constant `xSpeed` and `zRotation`.
    Requires the drive subsystem and accepts an `xSpeed` and `zRotation`.
    """

    def __init__(
        self, driveSubsystem: CANDriveSubsystem, xSpeed: float, zRotation: float
    ) -> None:
        super().__init__()

        self.driveSubsystem = driveSubsystem
        self.xSpeed = xSpeed
        self.zRotation = zRotation
        self.addRequirements(self.driveSubsystem)

    def execute(self) -> None:
        self.driveSubsystem.driveArcade(self.xSpeed, self.zRotation)

    def end(self, interrupted: bool) -> None:
        self.driveSubsystem.driveArcade(0, 0)

    def isFinished(self) -> bool:
        return False
