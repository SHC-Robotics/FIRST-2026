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
        leftPosition = self.climbSubsystem.leftArm.get_position().value
        if abs(leftPosition - self.climbSubsystem.initLeftPosition) < UPPER_BOUND_DELTA:
            return

        self.climbSubsystem.setVoltage(3)

    def end(self, interrupted: bool) -> None:
        self.climbSubsystem.stop()

    def isFinished(self) -> bool:
        leftPosition = self.climbSubsystem.leftArm.get_position().value
        # rightPosition = self.climbSubsystem.rightArm.get_position().
        print(f"leftPosition: {leftPosition}, initLeftPosition: {self.climbSubsystem.initLeftPosition}")
        # print(self.climbSubsystem.leftArm.get_voltage().value)
        # rightPosition = 1

        if abs(leftPosition - self.climbSubsystem.initLeftPosition) < UPPER_BOUND_DELTA:
            print("stopping climb down")
            return True

        return False

class ClimbDownManual(commands2.Command):
    def __init__(self, climbSubsystem: CANClimbSubsystem) -> None:
        super().__init__()

        self.climbSubsystem = climbSubsystem
        self.addRequirements(self.climbSubsystem)

    def initialize(self) -> None:
        print("going down manually")
        self.climbSubsystem.setVoltage(0.05)

    def end(self, interrupted: bool) -> None:
        self.climbSubsystem.stop()
