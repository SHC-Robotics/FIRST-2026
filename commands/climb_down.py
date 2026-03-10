# TODO: measure and configure
from subsystems.canclimbsubsystem import CANClimbSubsystem
import commands2


UPPER_BOUND_DELTA = 1

class ClimbDown(commands2.Command):
    def __init__(self, climbSubsystem: CANClimbSubsystem) -> None:
        super().__init__()

        self.climbSubsystem = climbSubsystem
        self.addRequirements(self.climbSubsystem)

    def initialize(self) -> None:
        if self.climbSubsystem.isAtBottom():
            print("ClimbDown: already at bottom, not starting")
            return

        self.targetPosition = self.climbSubsystem.homePosition
        print(f"ClimbDown: moving to home position {self.targetPosition:.2f} rotations")
        self.climbSubsystem.setMotionMagicPosition(self.targetPosition)

    def execute(self) -> None:
        # Stop each arm independently as soon as its limit switch triggers
        # This prevents over-driving whichever arm reaches the bottom first
        if self.climbSubsystem.isLeftAtBottom():
            self.climbSubsystem.stopLeft()
        if self.climbSubsystem.isRightAtBottom():
            self.climbSubsystem.stopRight()


    def end(self, interrupted: bool) -> None:
        self.climbSubsystem.stop()
        if interrupted:
            print("ClimbDown: interrupted")
        else:
            print("ClimbDown: reached home position")

    def isFinished(self) -> bool:
        # Primary stop: both limit switches physically confirm arms are fully down
        if self.climbSubsystem.isAtBottom():
            print("ClimbDown: both limit switches triggered — arms fully retracted, stopping")
            return True

        # Fallback: both encoders reached home within tolerance
        if self.climbSubsystem.isAtPosition(self.climbSubsystem.homePosition, ClimberConstants.POSITION_TOLERANCE):
            print("ClimbDown: home position reached")
            return True

        return False

class ClimbDownManual(commands2.Command):
    def __init__(self, climbSubsystem: CANClimbSubsystem) -> None:
        super().__init__()

        self.climbSubsystem = climbSubsystem
        self.addRequirements(self.climbSubsystem)

    def initialize(self) -> None:
        print("going down manually")
        self.climbSubsystem.setVoltage(2)

    def execute(self) -> None:
        # Stop each arm independently as soon as its limit switch triggers
        if self.climbSubsystem.isLeftAtBottom():
            self.climbSubsystem.stopLeft()
        if self.climbSubsystem.isRightAtBottom():
            self.climbSubsystem.stopRight()

    def end(self, interrupted: bool) -> None:
        self.climbSubsystem.stop()
