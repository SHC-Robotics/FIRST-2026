import math
from dataclasses import dataclass
from typing import Dict

from subsystems.vision_camera import VisionCamera
import wpilib
import commands2
from wpimath.geometry import Rotation2d, Translation3d, Pose2d, Translation2d


@dataclass
class CameraState:
    camera: VisionCamera
    minPercentFrame: float
    maxRotationSpeed: float


class VisionLocalizer(commands2.Subsystem):
    def __init__(self, drivetrain) -> None:
        super().__init__()

        self.drivetrain = drivetrain
        self.allowed = True
        self.cameras: Dict[str, CameraState] = dict()

        self._enabled = False

    def addCamera(
        self,
        camera: VisionCamera,
        minPercentFrame: float = 0.07,
        maxRotationSpeed: float = 120,
    ) -> None:
        self.cameras[camera.cameraName] = CameraState(
            camera,
            minPercentFrame,
            maxRotationSpeed,
        )
        camera.addLocalizer()

    def onEnabled(self) -> None:
        """
        Call from teleopInit() and autonomousInit() in robot.py.

        Switches LL4 to mode 4 (external IMU assisted convergence).
        IMU is already seeded from the disabled period — no seeding window needed.
        """
        self._enabled = True
        for c in self.cameras.values():
            c.camera.imuModeRequest.set(4)

    def onDisabled(self) -> None:
        """
        Call from disabledPeriodic() in robot.py once alliance is confirmed.

        Keeps the LL4 internal IMU continuously seeded from the NavX while
        disabled so it has an accurate heading reference before the match starts.
        Vision measurement fusion is blocked while disabled.
        """
        self._enabled = False
        for c in self.cameras.values():
            c.camera.imuModeRequest.set(1)

    def setAllowed(self, value: bool):
        self.allowed = value

    def periodic(self) -> None:
        if len(self.cameras) == 0:
            return

        heading = self.drivetrain.getPose().rotation()
        rotationSpeed = self.drivetrain.getRotationSpeed()

        # Always feed heading — required in both mode 1 (seeding) and mode 4
        for c in self.cameras.values():
            c.camera.robotOrientationSetRequest.set(
                [heading.degrees(), 0.0, 0.0, 0.0, 0.0, 0.0]
            )

        # Don't fuse vision measurements until enabled
        if not self._enabled or not self.allowed:
            return

        for c in self.cameras.values():
            camera = c.camera

            # Skip if rotating too fast — rapid turns smear tag corners
            if abs(rotationSpeed) > c.maxRotationSpeed:
                continue

            # Retrieve MT2 pose estimate
            visionPoseArray = camera.botPose.get()

            # Require full 10-element array before any indexing
            if len(visionPoseArray) < 10:
                continue

            tagCount = int(visionPoseArray[7])
            if tagCount == 0:
                continue

            # Correct latency-compensated timestamp
            # visionPoseArray[6] is pipeline latency in milliseconds
            latency_sec = visionPoseArray[6] / 1000.0
            timestamp = wpilib.Timer.getFPGATimestamp() - latency_sec

            visionX = visionPoseArray[0]
            visionY = visionPoseArray[1]  # index 1 is field Y, not index 2 (height)
            visionYaw = visionPoseArray[5]
            visionPose = Pose2d(
                Translation2d(visionX, visionY), Rotation2d.fromDegrees(visionYaw)
            )

            # Tighter std devs for multi-tag detections.
            # Always distrust vision rotation — gyro is more accurate for heading.
            xy_stdev = 0.3 if tagCount > 1 else 0.7
            self.drivetrain.poseEstimator.addVisionMeasurement(
                visionPose, timestamp, (xy_stdev, xy_stdev, 9999999)
            )

            wpilib.SmartDashboard.putString(
                "Vision Pose",
                f"x: {visionX:.2f}, y: {visionY:.2f}, yaw: {visionYaw:.1f}, tags: {tagCount}",
            )
