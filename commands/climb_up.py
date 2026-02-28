import commands2
from subsystems.canclimbsubsystem import CANClimbSubsystem
import wpilib

# TODO: measure and configure
LOWER_BOUND_DELTA = 10

class ClimbUp(commands2.Command):
    """
    A Command that represents the complete action of climbing up the tower.
    """

    def __init__(self, climbSubsystem: CANClimbSubsystem) -> None:
        super().__init__()

        self.climbSubsystem = climbSubsystem
        self.addRequirements(self.climbSubsystem)

    def initialize(self) -> None:
        self.climbSubsystem.setVoltage(0.1)

    def end(self, interrupted: bool) -> None:
        self.climbSubsystem.stop()

    def isFinished(self) -> bool:
        leftPosition = self.climbSubsystem.leftArm.get_position().value
        # rightPosition = self.climbSubsystem.rightArm.get_position().
        print(f"leftPosition: {leftPosition}, initLeftPosition: {self.climbSubsystem.initLeftPosition}")
        # print(self.climbSubsystem.leftArm.get_voltage().value)
        # rightPosition = 0

        if abs(leftPosition - self.climbSubsystem.initLeftPosition) > LOWER_BOUND_DELTA:
            print("stopping climb up")
            return True

        return False

class ClimbUpManual(commands2.Command):
    def __init__(self, climbSubsystem: CANClimbSubsystem) -> None:
        super().__init__()

        self.climbSubsystem = climbSubsystem
        self.addRequirements(self.climbSubsystem)

    def initialize(self) -> None:
        self.climbSubsystem.setVoltage(0.01)

    def end(self, interrupted: bool) -> None:
        self.climbSubsystem.stop()
