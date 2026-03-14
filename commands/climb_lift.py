import commands2
from constants import ClimberConstants
from subsystems.canclimbsubsystem import CANClimbSubsystem

class ClimbLift(commands2.Command):
    def __init__(self, climbSubsystem: CANClimbSubsystem) -> None:
        super().__init__()

        self.climbSubsystem = climbSubsystem
        self.addRequirements(self.climbSubsystem)

    def initialize(self) -> None:
        # if self.climbSubsystem.isAtBottom():
        #     print("ClimbLift: already at bottom, not starting")
        #     return

        self.targetPosition = self.climbSubsystem.homePosition - ClimberConstants.CLIMB_LIFT_TARGET
        print(f"ClimbLift: moving to home position {self.targetPosition:.2f} rotations")
        self.climbSubsystem.setMotionMagicPosition(self.targetPosition)

    # def execute(self) -> None:
    #     # Stop each arm independently as soon as its limit switch triggers
    #     # This prevents over-driving whichever arm reaches the bottom first
    #     if self.climbSubsystem.isLeftAtBottom():
    #         self.climbSubsystem.stopLeft()

    #     if self.climbSubsystem.isRightAtBottom():
    #         self.climbSubsystem.stopRight()


    def end(self, interrupted: bool) -> None:
        self.climbSubsystem.stop()

        if interrupted:
            print("ClimbLift: interrupted")
        else:
            # self.climbSubsystem.leftArm.set_position(0)
            # self.climbSubsystem.leftArm.set_position(0)

            print("ClimbLift: reached home position")

    def isFinished(self) -> bool:
        # # Primary stop: both limit switches physically confirm arms are fully down
        # if self.climbSubsystem.isAtBottom():
        #     print("ClimbLift: both limit switches triggered arms fully retracted, stopping")
        #     return True

        # # Fallback: both encoders reached home within tolerance
        # if self.climbSubsystem.isAtPosition(self.targetPosition, ClimberConstants.POSITION_TOLERANCE):
        #     print("ClimbLift: home position reached")
        #     return True

        return False
