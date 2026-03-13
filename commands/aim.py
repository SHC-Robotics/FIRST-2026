import commands2
import wpilib

from constants import AprilTagIds, FuelConstants
from subsystems.candrivesubsystem import CANDriveSubsystem

ALIGNED_THRESHOLD = 0.05

# All tag IDs that are part of each alliance's hub
RED_HUB_TAGS  = AprilTagIds.RED_HUB_TAGS   # e.g. [1, 2, 3, 4]  — update to match your constants
BLUE_HUB_TAGS = AprilTagIds.BLUE_HUB_TAGS  # e.g. [5, 6, 7, 8]


class Aim(commands2.Command):
    """
    Rotates the robot toward the closest visible hub AprilTag on its alliance side.
    Uses rawfiducials area (larger area = closer tag) to pick the best target.
    """

    def __init__(self, driveSubsystem: CANDriveSubsystem, driverController, visionCamera) -> None:
        super().__init__()

        self.driveSubsystem = driveSubsystem
        self.controller = driverController
        self.camera = visionCamera
        self.addRequirements(self.driveSubsystem)

    # ------------------------------------------------------------------
    # Helpers
    # ------------------------------------------------------------------

    def _getAllianceTags(self) -> list[int]:
        if wpilib.DriverStation.getAlliance() == wpilib.DriverStation.Alliance.kRed:
            return RED_HUB_TAGS
        return BLUE_HUB_TAGS

    def _getBestTagData(self) -> dict | None:
        best = None
        best_dist = float("inf")

        for tag_id in self._getAllianceTags():
            data = self.camera.getHubData(tag_id)
            if data["dist"] is not None and data["dist"] < best_dist:
                best_dist = data["dist"]
                best = {"id": tag_id, **data}

        return best

    # ------------------------------------------------------------------
    # Command lifecycle
    # ------------------------------------------------------------------

    def initialize(self) -> None:
        wpilib.SmartDashboard.putString("Current command", "Aim")
        print(f"Aim started — targeting closest of tags {self._getAllianceTags()}")

    def execute(self) -> None:
        best = self._getBestTagData()

        if best is None:
            self.driveSubsystem.driveArcade(0, 0)
            wpilib.SmartDashboard.putString("Aim Target", "NONE")
            return

        wpilib.SmartDashboard.putString(
            "Aim Target", f"id={best['id']} tx={best['tx']:.2f} area={best['area']:.3f}"
        )

        rotation_output = self.driveSubsystem.orientationController.calculate(best["tx"], 0)
        rotation_output = max(min(rotation_output, 0.15), -0.15)
        self.driveSubsystem.driveArcade(0, rotation_output)

    def isFinished(self) -> bool:
        best = self._getBestTagData()

        # No tag visible — bail out
        if best is None:
            return True

        return -ALIGNED_THRESHOLD < best["tx"] < ALIGNED_THRESHOLD

    def end(self, interrupted: bool) -> None:
        self.driveSubsystem.driveArcade(0, 0)
        print(f"Aim ended (interrupted={interrupted})")
