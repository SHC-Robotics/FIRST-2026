import math
from dataclasses import dataclass
from typing import Dict

from subsystems.vision_camera import VisionCamera
import wpilib
import commands2
from wpimath.geometry import Rotation2d, Translation3d, Pose2d, Translation2d

IMU_SEED_DURATION = 2.0  # seconds to spend in mode 1 seeding before switching to mode 4


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
        self.allowed = True
        self.cameras: Dict[str, CameraState] = dict()

        self._imuSeeding = False
        self._finishSeeding = False

    def addCamera(
        self,
        camera: VisionCamera,
        poseOnRobot: Translation3d,
        headingOnRobot: Rotation2d,
        pitchAngleDegrees: float,
        minPercentFrame: float = 0.07,
        maxRotationSpeed: float = 120,
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

    def onEnabled(self) -> None:
        """
        Call from teleopInit() and autonomousInit() in robot.py.

        Starts the IMU seeding sequence — mode 1 seeds the LL4 internal IMU
        from the NavX heading for IMU_SEED_DURATION seconds, then switches
        to mode 4 (external IMU assisted convergence) for the rest of the match.
        Vision measurements are blocked during the seeding window.
        """
        self._finishSeeding = True
        for c in self.cameras.values():
            c.camera.imuModeRequest.set(1)

    def onDisabled(self) -> None:
        """
        Call from disabledInit() in robot.py.

        Keeps the LL4 internal IMU continuously seeded from the NavX while
        disabled so it has an accurate heading reference before the match starts.
        """
        self._imuSeeding = True
        for c in self.cameras.values():
            c.camera.imuModeRequest.set(1)

    def setAllowed(self, value: bool):
        self.allowed = value

    def periodic(self) -> None:
        if len(self.cameras) == 0:
            return

        heading = self.drivetrain.getHeading()
        rotationSpeed = self.drivetrain.getRotationSpeed()

        # IMU seeding phase — push heading to cameras in mode 1 but skip
        # vision measurement fusion until the LL4 IMU is properly seeded.
        if self._imuSeeding:
            print("Seeding IMU")
            for c in self.cameras.values():
                c.camera.robotOrientationSetRequest.set(
                    [heading.degrees(), 0.0, 0.0, 0.0, 0.0, 0.0]
                )
            if self._finishSeeding:
                for c in self.cameras.values():
                    c.camera.imuModeRequest.set(4)
                self._imuSeeding = False
                self._finishSeeding = False
                print(f"LL4 IMU seeding complete after, switching to mode 4")
            return

        for c in self.cameras.values():
            camera = c.camera

            # Feed current robot heading to MT2 every loop.
            # MT2 requires this to eliminate pose ambiguity.
            # Do NOT push camerapose_robotspace_set here — set in web UI instead.
            camera.robotOrientationSetRequest.set(
                [heading.degrees(), 0.0, 0.0, 0.0, 0.0, 0.0]
            )

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

            visionX   = visionPoseArray[0]
            visionY   = visionPoseArray[1]   # index 1 is field Y, not index 2 (height)
            visionYaw = visionPoseArray[5]
            visionPose = Pose2d(
                Translation2d(visionX, visionY),
                Rotation2d.fromDegrees(visionYaw)
            )

            # Tighter std devs for multi-tag detections.
            # Always distrust vision rotation — gyro is more accurate for heading.
            xy_stdev = 0.3 if tagCount > 1 else 0.7
            self.drivetrain.poseEstimator.addVisionMeasurement(
                visionPose, timestamp, (xy_stdev, xy_stdev, 9999999)
            )

            wpilib.SmartDashboard.putString(
                "Vision Pose", f"x: {visionX:.2f}, y: {visionY:.2f}, yaw: {visionYaw:.1f}, tags: {tagCount}"
            )
