from subsystems.canclimbsubsystem import CANClimbSubsystem
import commands2


class ClimbManual(commands2.Command):
    def __init__(self, climbSubsystem: CANClimbSubsystem, climbController) -> None:
        super().__init__()

        self.climbSubsystem = climbSubsystem
        self.controller = climbController
        self.addRequirements(self.climbSubsystem)

    def execute(self) -> None:
        leftY = self.controller.getLeftY()
        if leftY < 0.1 and leftY > -0.1:
            self.climbSubsystem.stopLeft()
        else:
            self.climbSubsystem.setVoltageLeft(leftY)

        # rightY = self.controller.getRightY()
        # if rightY < 0.1 and rightY > -0.1:
        #     self.climbSubsystem.stopRight()
        # else:
        #     self.climbSubsystem.setVoltageRight(rightY)

    def end(self, interrupted: bool) -> None:
        self.climbSubsystem.stop()

    def isFinished(self) -> bool:
        return False
