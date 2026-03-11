import typing

import hal
import commands2

import robotcontainer

import sysidbot
import wpilib


class Robot(commands2.TimedCommandRobot):
    """
    Command-based robots should inherit from TimedCommandRobot, which runs a scheduler and includes
    an implementation of robotPeriodic.
    """

    def robotInit(self) -> None:
        # Instantiate the RobotContainer, which contains the majority of robot logic
        # (includes all button bindings and adds autonomous chooser to dashboard)
        self.sysidbot = sysidbot.SysIdRoutineBot()

        # Track usage of Kitbot code
        hal.report(hal.tResourceType.kResourceType_Framework, 10)

    def testInit(self) -> None:
        # Cancel all running commands when testing starts
        commands2.CommandScheduler.getInstance().cancelAll()
