from subsystems.candrivesubsystem import CANDriveSubsystem
from subsystems.vision_localizer import VisionLocalizer
import commands2


class Aim(commands2.Command):
    def __init__(self, driveSubsystem: CANDriveSubsystem) -> None:
        super().__init__()

        self.driveSubsystem = driveSubsystem
        self.addRequirements(self.driveSubsystem)

    def initialize(self) -> None:
        pass

    def execute(self) -> None:
        pass

    def isFinished(self) -> bool:
        return False

    def end(self, interrupted: bool) -> None:
        pass
