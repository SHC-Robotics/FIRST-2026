from subsystems.candrivesubsystem import CANDriveSubsystem
from wpimath.geometry import Rotation2d
import commands2


class ResetRotation(commands2.Command):
    def __init__(self, driveSubsystem: CANDriveSubsystem, rot: Rotation2d) -> None:
        super().__init__()

        self.driveSubsystem = driveSubsystem
        self.rot = rot
        self.addRequirements(self.driveSubsystem)

    def initialize(self) -> None:
        gyro_rot = self.driveSubsystem.getHeading()
        self.driveSubsystem.poseEstimator.resetRotation(gyro_rot.rotateBy(self.rot))

    def end(self, interrupted: bool) -> None:
        pass
