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
        self.targetPosition = (
            self.climbSubsystem.homePosition - ClimberConstants.CLIMB_UP_DELTA
        )
        print(f"ClimbUp: moving to position {self.targetPosition:.2f} rotations")
        self.climbSubsystem.setMotionMagicPosition(self.targetPosition)

    def execute(self) -> None:
        if self.climbSubsystem.isAtBottom():
            self.climbSubsystem.stop()

    def end(self, interrupted: bool) -> None:
        if interrupted:
            self.climbSubsystem.stop()
            print("ClimbUp: interrupted")
        else:
            # On clean finish, leave Motion Magic active so it holds position
            print("ClimbUp: reached target position")

    def isFinished(self) -> bool:
        if self.climbSubsystem.isAtBottom():
            print("ClimbUp: limit switch triggered, stopping")
            return True

        if self.climbSubsystem.isAtPosition(
            self.targetPosition, ClimberConstants.POSITION_TOLERANCE
        ):
            print("ClimbUp: target position reached")
            return True

        return False


class ClimbUpManual(commands2.Command):
    """
    Applies direct voltage to move the climber upward while the button is held.
    Stops immediately on button release. Bind with whileTrue() on your controller.
    """

    def __init__(self, climbSubsystem: CANClimbSubsystem) -> None:
        super().__init__()
        self.climbSubsystem = climbSubsystem
        self.addRequirements(self.climbSubsystem)

    def initialize(self) -> None:
        print("ClimbUpManual: started")
        self.climbSubsystem.setVoltage(-0.05)

    def execute(self) -> None:
        if self.climbSubsystem.isAtBottom():
            self.climbSubsystem.stop()

    def end(self, interrupted: bool) -> None:
        self.climbSubsystem.stop()
        print("ClimbUpManual: stopped")

    def isFinished(self) -> bool:
        return self.climbSubsystem.isAtBottom()
