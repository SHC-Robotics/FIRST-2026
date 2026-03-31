import commands2
from subsystems.canclimbsubsystem import CANClimbSubsystem
from constants import ClimberConstants

class ClimbHomeManual(commands2.Command):
    def __init__(self, climbSubsystem: CANClimbSubsystem) -> None:
        super().__init__()

        self.climbSubsystem = climbSubsystem
        self.addRequirements(self.climbSubsystem)

    def initialize(self) -> None:
        self.climbSubsystem.leftArm.set_position(0)
        self.climbSubsystem.rightArm.set_position(0)
        self.climbSubsystem.homePosition = 0
