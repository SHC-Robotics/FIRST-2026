import commands2
from subsystems.canclimbsubsystem import CANClimbSubsystem
import wpilib

# TODO: measure and configure
MOTOR_POSITION_DELTA = 20

class ClimbUp(commands2.Command):
    """
    A Command that represents the complete action of climbing up the tower.
    """

    def __init__(self, climbSubsystem: CANClimbSubsystem) -> None:
        super().__init__()

        self.climbSubsystem = climbSubsystem
        self.addRequirements(self.climbSubsystem)

    def initialize(self) -> None:
        self.initLeftPosition = self.climbSubsystem.leftArm.get_position().value
        # self.initRightPosition = self.climbSubsystem.rightArm.get_position().value

        self.climbSubsystem.setVoltage(8)

    def end(self, interrupted: bool) -> None:
        self.climbSubsystem.stop()

    def isFinished(self) -> bool:
        leftPosition = self.climbSubsystem.leftArm.get_position().value
        # rightPosition = self.climbSubsystem.rightArm.get_position().
        print(leftPosition)
        # print(self.climbSubsystem.leftArm.get_voltage().value)
        # rightPosition = 0

        if abs(leftPosition - self.initLeftPosition) > MOTOR_POSITION_DELTA:
            return True

        return False
