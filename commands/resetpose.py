from subsystems.candrivesubsystem import CANDriveSubsystem
from wpimath.geometry import Rotation2d
import commands2
import wpilib


class ResetRotation(commands2.Command):
    def __init__(self, driveSubsystem: CANDriveSubsystem, rot: Rotation2d) -> None:
        super().__init__()

        self.driveSubsystem = driveSubsystem
        self.rot = rot
        self.addRequirements(self.driveSubsystem)

    def initialize(self) -> None:
        self.driveSubsystem.gyro.setAngleAdjustment(self.rot.degrees())
        self.driveSubsystem.gyro.zeroYaw()

    def end(self, interrupted: bool) -> None:
        pass

class ZeroRotation(commands2.Command):
    def __init__(self, driveSubsystem: CANDriveSubsystem) -> None:
        super().__init__()

        self.driveSubsystem = driveSubsystem
        self.addRequirements(self.driveSubsystem)

    def initialize(self) -> None:
        if wpilib.DriverStation.getAlliance() == wpilib.DriverStation.Alliance.kRed:
            self.driveSubsystem.gyro.setAngleAdjustment(180)
        else:
            self.driveSubsystem.gyro.setAngleAdjustment(0)

        self.driveSubsystem.gyro.zeroYaw()

    def isFinished(self) -> None:
        return True
