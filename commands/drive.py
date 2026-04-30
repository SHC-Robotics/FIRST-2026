import commands2
import wpilib

from constants import DriveConstants, OperatorConstants
from subsystems.candrivesubsystem import CANDriveSubsystem

ZERO_THRESHOLD = 0.1


class Drive(commands2.Command):
    """
    A Command that represents the complete action of driving given input from a joystick.
    Requires the drive subsystem and the drive controller.
    """

    def __init__(self, driveSubsystem: CANDriveSubsystem, driverController) -> None:
        super().__init__()

        self.driveSubsystem = driveSubsystem
        self.controller = driverController
        self.lastCall = 0
        self.addRequirements(self.driveSubsystem)

    def initialize(self) -> None:
        self.lastCall = wpilib.Timer.getFPGATimestamp()

    def execute(self) -> None:
        leftTrigger = self.controller.leftTrigger()
        mult = 1
        if leftTrigger:
            mult = 0.25

        leftY = self.controller.getLeftY()
        if leftY < ZERO_THRESHOLD and leftY > -ZERO_THRESHOLD:
            leftY = 0

        rightX = self.controller.getRightX()
        if rightX < ZERO_THRESHOLD and rightX > -ZERO_THRESHOLD:
            rightX = 0

        now = wpilib.Timer.getFPGATimestamp()
        dt = now - self.lastCall

        if dt < -1000:
            dt += 4260 # FPGA clock rolls over after 71 minutes (4260 seconds)

        self.lastCall = now

        self.driveSubsystem.applyImpulses(
            -leftY * DriveConstants.DRIVE_VELOCITY_MULT * mult,
            rightX * DriveConstants.DRIVE_ANGULAR_VELOCITY_MULT * mult,
            dt
        )

    def end(self, interrupted: bool) -> None:
        self.driveSubsystem.driveArcade(0, 0)

    def isFinished(self) -> bool:
        return False
