import wpilib
import commands2
import commands2.button
import commands2.cmd

from constants import OperatorConstants
from commands.drive import Drive
from commands.eject import Eject
from commands.exampleauto import ExampleAuto
from commands.intake import Intake
from commands.launchsequence import LaunchSequence
from subsystems.candrivesubsystem import CANDriveSubsystem
from subsystems.canfuelsubsystem import CANFuelSubsystem


class RobotContainer:
    def __init__(self) -> None:
        # The robot's subsystems
        self.driveSubsystem = CANDriveSubsystem()
        self.fuelSubsystem = CANFuelSubsystem()

        # The driver's controller
        self.driverController = commands2.button.CommandXboxController(
            OperatorConstants.DRIVER_CONTROLLER_PORT
        )

        # The operator's controller
        self.operatorController = commands2.button.CommandXboxController(
            OperatorConstants.OPERATOR_CONTROLLER_PORT
        )

        # The autonomous chooser
        self.autoChooser = wpilib.SendableChooser()

        self.configureBindings()

        # Set the options to show up in the Dashboard for selecting auto modes.
        self.autoChooser.setDefaultOption(
            "Autonomous", ExampleAuto(self.driveSubsystem, self.fuelSubsystem)
        )
        wpilib.SmartDashboard.putData("Autonomous", self.autoChooser)

    def configureBindings(self) -> None:
        # While the left bumper on operator controller is held, intake Fuel
        self.operatorController.leftBumper().whileTrue(Intake(self.fuelSubsystem))
        # While the right bumper on the operator controller is held, spin up for 1
        # second, then launch fuel. When the button is released, stop.
        self.operatorController.rightBumper().whileTrue(
            LaunchSequence(self.fuelSubsystem)
        )
        # While the A button is held on the operator controller, eject fuel back out
        # the intake
        self.operatorController.a().whileTrue(Eject(self.fuelSubsystem))

        # Set the default command for the drive subsystem to the command provided by
        # factory with the values provided by the joystick axes on the driver
        # controller.
        self.driveSubsystem.setDefaultCommand(
            Drive(self.driveSubsystem, self.driverController)
        )

        self.fuelSubsystem.run(lambda: self.fuelSubsystem.stop())

    def getAutonomousCommand(self) -> commands2.Command:
        return self.autoChooser.getSelected()
