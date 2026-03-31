import commands2
from subsystems.canclimbsubsystem import CANClimbSubsystem
from constants import ClimberConstants


class ClimbUp(commands2.Command):
    """
    Moves the climber arms to the fully extended (up) position using Motion Magic.
    The motor follows a profiled path capped at MM_CRUISE_VELOCITY, then holds position.
    Finishes when the arms arrive within tolerance of the target.
    """

    def __init__(self, climbSubsystem: CANClimbSubsystem) -> None:
        super().__init__()
        self.climbSubsystem = climbSubsystem
        self.addRequirements(self.climbSubsystem)

    def initialize(self) -> None:
        self.targetPosition = self.climbSubsystem.homePosition - ClimberConstants.CLIMB_UP_DELTA
        print(f"ClimbUp: moving to position {self.targetPosition:.2f} rotations")
        self.climbSubsystem.setMotionMagicPosition(self.targetPosition)

    def execute(self) -> None:
        if self.climbSubsystem.isLeftAtPosition(self.targetPosition, ClimberConstants.POSITION_TOLERANCE):
            self.climbSubsystem.stopLeft()

        if self.climbSubsystem.isRightAtPosition(self.targetPosition, ClimberConstants.POSITION_TOLERANCE):
            self.climbSubsystem.stopRight()

    def end(self, interrupted: bool) -> None:
        if interrupted:
            self.climbSubsystem.stop()
            print("ClimbUp: interrupted")
        else:
            # On clean finish, leave Motion Magic active so it holds position
            print("ClimbUp: reached target position")

    def isFinished(self) -> bool:
        if self.climbSubsystem.isAtPosition(self.targetPosition, ClimberConstants.POSITION_TOLERANCE):
            print("ClimbUp: target position reached")
            return True

        return False
