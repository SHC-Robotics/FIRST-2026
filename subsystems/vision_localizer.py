import math
from dataclasses import dataclass
from typing import Dict

from subsystems.vision_camera import VisionCamera
import wpilib
import commands2
from wpimath.geometry import Rotation2d, Translation3d, Pose2d, Translation2d

IMU_SEED_DURATION = 2.0  # seconds to spend in mode 1 before switching to mode 4


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

        self._imuSeeded = False
        self._seedStartTime = 0.0

    def addCamera(
        self,
        camera: VisionCamera,
        poseOnRobot: Translation3d,
        headingOnRobot: Rotation2d,
        pitchAngleDegrees: float,
        minPercentFrame: float = 0.07,
        maxRotationSpeed: float = 120,  # degrees per second
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
        Starts the IMU seeding sequence.
        """
        self._seedStartTime = wpilib.Timer.getFPGATimestamp()
        self._imuSeeded = False
        for c in self.cameras.values():
            c.camera.imuModeRequest.set(1)

    def onDisabled(self) -> None:
        """
        Call from disabledInit() in robot.py.
        Keeps LL4 IMU seeded from NavX while disabled.
        """
        for c in self.cameras.values():
            c.camera.imuModeRequest.set(1)

    def setAllowed(self, value: bool) -> None:
        self.allowed = value

    def periodic(self) -> None:
        # Skip vision entirely in simulation — no Limelight available
        if wpilib.RobotBase.isSimulation():
            return

        if len(self.cameras) == 0:
            return

        heading      = self.drivetrain.getHeading()
        rotationSpeed = self.drivetrain.getRotationSpeed()  # deg/s, CCW positive

        # IMU seeding phase — push heading in mode 1 but skip vision fusion
        if not self._imuSeeded:
            elapsed = wpilib.Timer.getFPGATimestamp() - self._seedStartTime
            wpilib.SmartDashboard.putNumber("IMU Seed Elapsed", elapsed)
            for c in self.cameras.values():
                c.camera.robotOrientationSetRequest.set(
                    [heading.degrees(), 0.0, 0.0, 0.0, 0.0, 0.0]
                )
            if elapsed >= IMU_SEED_DURATION:
                for c in self.cameras.values():
                    c.camera.imuModeRequest.set(4)
                self._imuSeeded = True
                print(f"LL4 IMU seeding complete after {elapsed:.2f}s, switching to mode 4")
            return

        for c in self.cameras.values():
            camera = c.camera

            # Feed heading to MT2 every loop — required to resolve pose ambiguity
            camera.robotOrientationSetRequest.set(
                [heading.degrees(), rotationSpeed, 0.0, 0.0, 0.0, 0.0]
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

            # Latency-compensated timestamp
            latency_sec = visionPoseArray[6] / 1000.0
            timestamp   = wpilib.Timer.getFPGATimestamp() - latency_sec

            visionX   = visionPoseArray[0]
            visionY   = visionPoseArray[1]  # index 1 is field Y, not index 2 (height)
            visionYaw = visionPoseArray[5]
            visionPose = Pose2d(
                Translation2d(visionX, visionY),
                Rotation2d.fromDegrees(visionYaw)
            )

            # Per-measurement std devs — always distrust vision rotation
            xy_stdev = 0.3 if tagCount > 1 else 0.7
            self.drivetrain.poseEstimator.addVisionMeasurement(
                visionPose, timestamp, (xy_stdev, xy_stdev, 9999999)
            )

            wpilib.SmartDashboard.putString(
                "Vision Pose",
                f"x: {visionX:.2f}, y: {visionY:.2f}, yaw: {visionYaw:.1f}, tags: {tagCount}"
            )