from constants import FieldConstants
from subsystems.candrivesubsystem import CANDriveSubsystem
from subsystems.vision_localizer import VisionLocalizer
import commands2
import math
import wpilib
from wpimath.geometry import Translation2d, Rotation2d, Pose2d


class Aim(commands2.Command):
    def __init__(self, driveSubsystem: CANDriveSubsystem) -> None:
        super().__init__()

        self.driveSubsystem = driveSubsystem
        self.addRequirements(self.driveSubsystem)
        self.finished = False

    def initialize(self) -> None:
        pass

    def execute(self) -> None:
        delta_rot = self.compute_delta_rot()

        wpilib.SmartDashboard.putNumber("delta rot degrees", delta_rot)

        if abs(delta_rot) < 1.0:
            self.finished = True
            return

        rotation_output = self.driveSubsystem.orientationController.calculate(delta_rot, 0)
        rotation_output = max(min(rotation_output, 0.15), -0.15)
        self.driveSubsystem.driveArcade(0, rotation_output)

    def isFinished(self) -> bool:
        return self.finished

    def end(self, interrupted: bool) -> None:
        self.driveSubsystem.driveArcade(0, 0)

    def compute_delta_rot(self):
        if wpilib.DriverStation.getAlliance() == wpilib.DriverStation.Alliance.kRed:
            hub_position = FieldConstants.RED_HUB_POSITION
        else:
            hub_position = FieldConstants.BLUE_HUB_POSITION

        bot_pose = self.driveSubsystem.getPose()
        to_hub = hub_position - bot_pose.translation()
        hub_distance = math.sqrt(to_hub.dot(to_hub))
        target_rot = (to_hub / hub_distance).angle()
        delta_rot = target_rot.degrees() - bot_pose.rotation().degrees()

        if delta_rot > 180:
            delta_rot -= 360

        return delta_rot
