import commands2
from constants import ClimberConstants
from subsystems.canclimbsubsystem import CANClimbSubsystem

class ClimbLift(commands2.Command):
    def __init__(self, climbSubsystem: CANClimbSubsystem) -> None:
        super().__init__()

        self.climbSubsystem = climbSubsystem
        self.addRequirements(self.climbSubsystem)

    def initialize(self) -> None:
        self.targetPosition = self.climbSubsystem.homePosition - ClimberConstants.CLIMB_LIFT_TARGET
        print(f"ClimbLift: moving to home position {self.targetPosition:.2f} rotations")
        self.climbSubsystem.setMotionMagicPosition(self.targetPosition)

    def end(self, interrupted: bool) -> None:
        self.climbSubsystem.stop()

        if interrupted:
            print("ClimbLift: interrupted")
        else:
            print("ClimbLift: reached home position")

    def isFinished(self) -> bool:
        return False
