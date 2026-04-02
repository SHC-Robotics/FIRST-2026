# TODO: measure and configure
from subsystems.canclimbsubsystem import CANClimbSubsystem
from constants import ClimberConstants
import commands2


UPPER_BOUND_DELTA = 1

class ClimbDown(commands2.Command):
    def __init__(self, climbSubsystem: CANClimbSubsystem) -> None:
        super().__init__()

        self.climbSubsystem = climbSubsystem
        self.addRequirements(self.climbSubsystem)

    def initialize(self) -> None:
        self.targetPosition = self.climbSubsystem.homePosition
        print(f"ClimbDown: moving to home position {self.targetPosition:.2f} rotations")
        self.climbSubsystem.setMotionMagicPosition(self.targetPosition)

    def execute(self) -> None:
        # Stop each arm independently as soon as its limit switch triggers
        # This prevents over-driving whichever arm reaches the bottom first
        if self.climbSubsystem.isLeftAtPosition(self.targetPosition, ClimberConstants.POSITION_TOLERANCE):
            self.climbSubsystem.stopLeft()

        if self.climbSubsystem.isRightAtPosition(self.targetPosition, ClimberConstants.POSITION_TOLERANCE):
            self.climbSubsystem.stopRight()

    def end(self, interrupted: bool) -> None:
        self.climbSubsystem.stop()

        if interrupted:
            print("ClimbDown: interrupted")
        else:
            print("ClimbDown: reached home position")

    def isFinished(self) -> bool:
        # Stop: both encoders reached home within tolerance
        if self.climbSubsystem.isAtPosition(self.targetPosition, ClimberConstants.POSITION_TOLERANCE):
            return True

        return False
