import typing

import hal
import commands2

import robotcontainer


class Robot(commands2.TimedCommandRobot):
    def robotInit(self) -> None:
        self.autonomousCommand: typing.Optional[commands2.Command] = None
        self.container = robotcontainer.RobotContainer()
        hal.report(hal.tResourceType.kResourceType_Framework, 10)

    def autonomousInit(self) -> None:
        self.autonomousCommand = self.container.getAutonomousCommand()

        if self.autonomousCommand is not None:
            self.autonomousCommand.schedule()

    def teleopInit(self) -> None:
        if self.autonomousCommand is not None:
            self.autonomousCommand.cancel()

    def testInit(self) -> None:
        commands2.CommandScheduler.getInstance().cancelAll()
