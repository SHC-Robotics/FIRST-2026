import commands2
from subsystems.canclimbsubsystem import CANClimbSubsystem
import wpilib

# TODO: measure and configure
FINISHED_CLIMBING_POSITION = 1

class ClimbUp(commands2.Command):
    """
    A Command that represents the complete action of climbing up the tower.
    """

    def __init__(self, climbSubsystem: CANClimbSubsystem) -> None:
        super().__init__()

        self.climbSubsystem = climbSubsystem
        self.addRequirements(self.climbSubsystem)

    def initialize(self) -> None:
        self.climbSubsystem.setVoltage(8)

    def end(self, interrupted: bool) -> None:
        self.climbSubsystem.stop()

    def isFinished(self) -> bool:
        leftPosition = self.climbSubsystem.leftArm.get_position().value
        # rightPosition = self.climbSubsystem.rightArm.get_position().
        print(leftPosition)
        # print(self.climbSubsystem.leftArm.get_voltage().value)
        # rightPosition = 0

        # if leftPosition > FINISHED_CLIMBING_POSITION or rightPosition > FINISHED_CLIMBING_POSITION:
        #     return True

        return False
