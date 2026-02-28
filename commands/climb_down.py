# TODO: measure and configure
from subsystems.canclimbsubsystem import CANClimbSubsystem
import commands2


MOTOR_POSITION_DELTA = 10

class ClimbDown(commands2.Command):
    def __init__(self, climbSubsystem: CANClimbSubsystem) -> None:
        super().__init__()

        self.climbSubsystem = climbSubsystem
        self.addRequirements(self.climbSubsystem)

    def initialize(self) -> None:
        self.initLeftPosition = self.climbSubsystem.leftArm.get_position().value
        # self.initRightPosition = self.climbSubsystem.rightArm.get_position().value

        self.climbSubsystem.setVoltage(-0.1)

    def end(self, interrupted: bool) -> None:
        self.climbSubsystem.stop()

    def isFinished(self) -> bool:
        leftPosition = self.climbSubsystem.leftArm.get_position().value
        # rightPosition = self.climbSubsystem.rightArm.get_position().
        print(f"leftPosition: {leftPosition}, initLeftPosition: {self.initLeftPosition}")
        # print(self.climbSubsystem.leftArm.get_voltage().value)
        # rightPosition = 1

        if abs(leftPosition - self.initLeftPosition) > MOTOR_POSITION_DELTA:
            print("stopping climb down")
            return True

        return False
