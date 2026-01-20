import typing

import hal
import commands2

import robotcontainer


class Robot(commands2.TimedCommandRobot):
    """
    Command-based robots should inherit from TimedCommandRobot, which runs a scheduler and includes
    an implementation of robotPeriodic.
    """

    def robotInit(self) -> None:
        self.autonomousCommand: typing.Optional[commands2.Command] = None

        # Instantiate the RobotContainer, which contains the majority of robot logic
        # (includes all button bindings and adds autonomous chooser to dashboard)
        self.container = robotcontainer.RobotContainer()

        # Track usage of Kitbot code
        hal.report(hal.tResourceType.kResourceType_Framework, 10)

    def autonomousInit(self) -> None:
        self.autonomousCommand = self.container.getAutonomousCommand()

        # Schedule the autonomous command
        if self.autonomousCommand is not None:
            self.autonomousCommand.schedule()

    def teleopInit(self) -> None:
        # Ensure autonomous stops running when teleop starts
        if self.autonomousCommand is not None:
            self.autonomousCommand.cancel()

    def testInit(self) -> None:
        # Cancel all running commands when testing starts
        commands2.CommandScheduler.getInstance().cancelAll()
