# TODO: measure and configure
from subsystems.canclimbsubsystem import CANClimbSubsystem


INITIAL_CLIMBING_POSITION = 0

class ClimbDown(commands2.Command):
    def __init__(self, climbSubsystem: CANClimbSubsystem) -> None:
        super().__init__()

        self.climbSubsystem = climbSubsystem
        self.addRequirements(self.climbSubsystem)

    def initialize(self) -> None:
        self.climbSubsystem.setVoltage(-1)

    def end(self, interrupted: bool) -> None:
        self.climbSubsystem.stop()

    def isFinished(self) -> bool:
        leftPosition = self.climbSubsystem.leftArm.get_position().value
        rightPosition = self.climbSubsystem.rightArm.get_position().value

        if leftPosition < INITIAL_CLIMBING_POSITION or rightPosition < INITIAL_CLIMBING_POSITION:
            return True

        return False
