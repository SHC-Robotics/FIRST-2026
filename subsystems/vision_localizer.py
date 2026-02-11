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
    def __init__(self, drivetrain, flipIfRed=False) -> None:
        super().__init__()

        self.drivetrain = drivetrain

        from getpass import getuser

        self.username = getuser()
        self.flipIfRed = flipIfRed

        self.learningRate = wpilib.SendableChooser()
        self.learningRate.addOption("1.0", 1.0)
        self.learningRate.addOption("0.3", 0.3)
        self.learningRate.addOption("0.1", 0.1)
        self.learningRate.addOption("0.03", 0.03)
        self.learningRate.addOption("0.01", 0.01)
        self.learningRate.addOption("0.001", 0.001)
        wpilib.SmartDashboard.putData("LocalLearnRate", self.learningRate)

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

    def setAllowed(self, value: bool):
        self.allowed = value

    def periodic(self) -> None:
        if len(self.cameras) == 0:
            return

        enabled, flipped = None, False
        if self.enabled is None:
            self.initEnabledChooser()
        if self.enabled is not None:
            enabled, flipped = self.enabled.getSelected()
        if not self.allowed:
            enabled = False

        if not enabled:
            return

        learningRate: float = LEARNING_RATE * self.learningRateMult.getSelected()
        odometryPos: Pose2d = self.drivetrain.getPose()
        heading: Rotation2d = self.drivetrain.getHeading()
        rotationSpeed: float = (
            self.drivetrain.getTurnRate()
        )  # rotation speed in degrees per second
        assert heading is not None

        for c in self.cameras.values():
            camera = c.camera
            if not camera.ticked or abs(rotationSpeed) > c.maxRotationSpeed:
                continue

            p = c.poseOnRobot
            camera.cameraPoseSetRequest.set(
                [
                    p.x,
                    p.y,
                    p.z,
                    c.pitchAngleDegrees,
                    0.0,
                    c.headingOnRobot.degrees(),
                ]
            )

            # Limelight4-only (does nothing on Limelight 3)
            camera.imuModeRequest.set(0)
            # 0 - use external imu (the only option available on Limelight 3)
            # 1 - use external imu, seed internal imu
            # 2 - use internal
            # 3 - use internal with MT1 assisted convergence
            # 4 - use internal IMU with external IMU assisted convergence

            if flipped:
                yaw = (heading + U_TURN).degrees()
                camera.robotOrientationSetRequest.set([yaw, 0.0, 0.0, 0.0, 0.0, 0.0])
                botpose = camera.botPoseFlipped.get()
            else:
                yaw = heading.degrees()
                camera.robotOrientationSetRequest.set([yaw, 0.0, 0.0, 0.0, 0.0, 0.0])
                botpose = camera.botPose.get()

            if len(botpose) >= 11:
                # Translation (X,Y,Z), Rotation(Roll,Pitch,Yaw) in degrees, total latency (cl+tl), tag count, tag span, average tag distance from camera, average tag area (percentage of image)
                (
                    x,
                    y,
                    z,
                    roll,
                    pitch,
                    yaw,
                    latencyMillisec,
                    count,
                    span,
                    distance,
                    percentage,
                ) = botpose[0:11]
                # SmartDashboard.putNumber("Localizer/" + c.camera.cameraName, percentage)
                if (
                    count > 0
                    and percentage > c.minPercentFrame
                    and not (x == 0 and y == 0)
                ):
                    gain = (
                        percentage / TYPICAL_PERCENT_FRAME
                    )  # tags nearby have more say than tags far away
                    if not EMPHASIZE_TAGS_NEARBY:
                        gain = math.sqrt(gain)
                    shift = Translation2d(x - odometryPos.x, y - odometryPos.y) * min(
                        learningRate * gain, 0.5
                    )
                    self.drivetrain.adjustOdometry(shift, Rotation2d.fromDegrees(0))

    def initEnabledChooser(self):
        flipped = None
        if self.username == "lvuser" and self.flipIfRed is not None:
            # if we are running on RoboRIO, wait until driver station gives us alliance color
            color = wpilib.DriverStation.getAlliance()
            if color is None:
                return  # we cannot yet decide on whether the field should be flipped
                flipped = (
                    color == wpilib.DriverStation.Alliance.kRed
                ) and self.flipIfRed
                print("Localizer: color={}, flipped={}".format(color, flipped))
            print(
                "Localizer will assume flipped={} (username={}, flipIfRed={})".format(
                    flipped, self.username, self.flipIfRed
                )
            )

            self.enabled = wpilib.SendableChooser()
            self.enabled.addOption("off", (None, False))
            if flipped in (None, False):
                self.enabled.setDefaultOption("on", (True, False))
            if flipped in (None, True):
                self.enabled.setDefaultOption("on-flipped", (True, True))
            wpilib.SmartDashboard.putData("Localizer", self.enabled)
