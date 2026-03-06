import math
from dataclasses import dataclass
from typing import Dict

from subsystems.vision_camera import VisionCamera
import wpilib
import commands2
from wpimath.geometry import Rotation2d, Translation3d, Pose2d, Translation2d

U_TURN = Rotation2d.fromDegrees(180)
LEARNING_RATE = 0.3
TYPICAL_PERCENT_FRAME = 0.7  # when the tag is ~2m away
EMPHASIZE_TAGS_NEARBY = False


@dataclass
class CameraState:
    camera: VisionCamera
    poseOnRobot: Translation3d
    headingOnRobot: Rotation2d
    pitchAngleDegrees: float
    minPercentFrame: float
    maxRotationSpeed: float


class VisionLocalizer(commands2.Subsystem):
    def __init__(self, drivetrain) -> None:
        super().__init__()

        self.drivetrain = drivetrain

        from getpass import getuser

        self.username = getuser()

        self.enabled = None
        self.allowed = True
        self.cameras: Dict[str, CameraState] = dict()

    def addCamera(
        self,
        camera: VisionCamera,
        poseOnRobot: Translation3d,
        headingOnRobot: Rotation2d,
        pitchAngleDegrees: float,
        minPercentFrame: float = 0.07,
        maxRotationSpeed: float = 120, # degrees per second
    ) -> None:
        self.cameras[camera.cameraName] = CameraState(
            camera,
            poseOnRobot,
            headingOnRobot,
            pitchAngleDegrees,
            minPercentFrame,
            maxRotationSpeed,
        )

        camera.addLocalizer()

    def setAllowed(self, value: bool):
        self.allowed = value

    def periodic(self) -> None:
        if len(self.cameras) == 0:
            return

        heading = self.drivetrain.getHeading()
        rotationSpeed = self.drivetrain.gyro.getRate()

        for c in self.cameras.values():
            camera = c.camera

            # Update network table values for MegaTag2
            p = c.poseOnRobot
            camera.cameraPoseSetRequest.set([p.x, p.y, p.z, c.pitchAngleDegrees, 0.0, c.headingOnRobot.degrees()])

            camera.imuModeRequest.set(4) # use internal IMU with external IMU assisted convergence

            yaw = heading.degrees()
            camera.robotOrientationSetRequest.set([yaw, 0.0, 0.0, 0.0, 0.0, 0.0])

            # Retrieve updated robot pose from MegaTag2 and add it to the drivetrain pose estimator
            visionPoseArray = camera.getBotPose()
            timestamp = camera.lastHeartbeatTime

            if len(visionPoseArray) == 0:
                continue

            if camera.getTv() == 0:
                continue

            if abs(rotationSpeed) > c.maxRotationSpeed:
                continue

            visionX = visionPoseArray[0]
            visionY = visionPoseArray[1]
            visionYaw = visionPoseArray[5]
            tagCount = int(visionPoseArray[7])
            visionPose = Pose2d(x=visionX, y=visionY, rotation=Rotation2d.fromDegrees(visionYaw))

            wpilib.SmartDashboard.putString("Vision Pose", f"x: {visionX}, y: {visionY}, yaw: {visionYaw}")

            xy_stdev = 0.3 if tagCount > 1 else 0.7
            self.drivetrain.poseEstimator.setVisionMeasurementStdDevs((xy_stdev, xy_stdev, 9999999))
            self.drivetrain.poseEstimator.addVisionMeasurement(visionPose, timestamp)
