from constants import FieldConstants
from subsystems.candrivesubsystem import CANDriveSubsystem
from subsystems.canfuelsubsystem import CANFuelSubsystem
from subsystems.vision_localizer import VisionLocalizer
import commands2
import math
import wpilib
from wpimath.geometry import Translation2d, Rotation2d, Pose2d


class Aim(commands2.Command):
    def __init__(self, driveSubsystem: CANDriveSubsystem, fuelSubsystem: CANFuelSubsystem) -> None:
        super().__init__()

        self.driveSubsystem = driveSubsystem
        self.fuelSubsystem = fuelSubsystem
        self.addRequirements(self.driveSubsystem)
        self.finished = False

        # Distance (m) -> Multiplier
        # Sorted least to greatest distance
        self.speeds = {
            5: 0.67,
            4: 0.65,
            3: 0.61,
            2.5: 0.58,
            2: 0.56,
        }

    def initialize(self) -> None:
        self.finished = False

    def execute(self) -> None:
        (distance, delta_rot) = self.compute_distance_delta_rot()
        speed = self.find_speed(distance)

        wpilib.SmartDashboard.putNumber("delta rot degrees", delta_rot)
        wpilib.SmartDashboard.putNumber("hub distance", distance)
        wpilib.SmartDashboard.putNumber("aim shoot mult", speed)

        self.fuelSubsystem.multiplier = speed

        if abs(delta_rot) < 1.5:
            self.finished = True
            return

        rotation_output = self.driveSubsystem.orientationController.calculate(delta_rot, 0)
        rotation_output = max(min(rotation_output, 0.15), -0.15)
        self.driveSubsystem.driveArcade(0, rotation_output)

    def isFinished(self) -> bool:
        return self.finished

    def end(self, interrupted: bool) -> None:
        self.driveSubsystem.driveArcade(0, 0)

    def compute_distance_delta_rot(self):
        if wpilib.DriverStation.getAlliance() == wpilib.DriverStation.Alliance.kRed:
            hub_position = FieldConstants.RED_HUB_POSITION
        else:
            hub_position = FieldConstants.BLUE_HUB_POSITION

        bot_pose = self.driveSubsystem.getPose()

        if bot_pose.X() > FieldConstants.NEUTRAL_ZONE_LOWER and bot_pose.X() < FieldConstants.NEUTRAL_ZONE_UPPER:
            if wpilib.DriverStation.getAlliance() == wpilib.DriverStation.Alliance.kRed:
                to_hub = Translation2d(FieldConstants.RED_ALLIANCE_ZONE_TARGET - bot_pose.X(), 0.0)
            else:
                to_hub = Translation2d(bot_pose.X() - FieldConstants.BLUE_ALLIANCE_ZONE_TARGET, 0.0)
        else:
            to_hub = hub_position - bot_pose.translation()

        hub_distance = math.sqrt(to_hub.dot(to_hub))
        target_rot = (to_hub / hub_distance).angle()
        delta_rot = target_rot.degrees() - bot_pose.rotation().degrees()

        if delta_rot > 180:
            delta_rot -= 360

        return (hub_distance, delta_rot)

    def find_speed(self, dist):
        distances = sorted(self.speeds.keys())

        if dist < distances[0]:
            return self.speeds[distances[0]]

        for i, d in enumerate(distances):
            if dist < d:
                d1 = distances[i - 1]
                d2 = d
                speed1 = self.speeds[d1]
                speed2 = self.speeds[d2]

                t = (dist - d1) / (d2 - d1)
                s = speed1 + (speed2 - speed1) * t
                return s

        return self.speeds[distances[-1]]
