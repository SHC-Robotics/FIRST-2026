from subsystems.vision_camera import VisionCamera
from subsystems.vision_localizer import VisionLocalizer
import wpilib
import commands2
import commands2.button
import commands2.cmd

from constants import OperatorConstants
from commands.drive import Drive
from commands.eject import Eject
from commands.aim import Aim
from commands.intake import Intake
from commands.climb_down import ClimbDown
from commands.climb_up import ClimbUp
from commands.launchsequence import LaunchSequence
from commands.launch import StopLaunch
from subsystems.candrivesubsystem import CANDriveSubsystem
from subsystems.canfuelsubsystem import CANFuelSubsystem
from subsystems.canclimbsubsystem import CANClimbSubsystem

from wpimath.geometry import Translation3d, Rotation2d
from pathplannerlib.auto import AutoBuilder, NamedCommands

class RobotContainer:
    """
    The robot container, which stores the robot's subsystems, controllers, binds buttons to commands,
    and manages autonomous modes.
    """

    def __init__(self) -> None:
        # The robot's subsystems.
        # A Subsystem is a collection of motors, sensors, and other hardware objects that are operated on by a Command.
        self.driveSubsystem = CANDriveSubsystem()
        self.fuelSubsystem = CANFuelSubsystem()
        self.climbSubsystem = CANClimbSubsystem()

        self.visionLocalizer = VisionLocalizer(self.driveSubsystem)
        self.frontVisionCamera = VisionCamera("limelight-front")
        self.backVisionCamera = VisionCamera("limelight-back")
        self.visionLocalizer.addCamera(
            self.frontVisionCamera,
            poseOnRobot=Translation3d(0.3066, 0.1056, 0.66), # Forward: +X, Right: +Y, Up: +Z
            headingOnRobot=Rotation2d(0.0), # yaw
            pitchAngleDegrees=0.0,
        )
        self.visionLocalizer.addCamera(
            self.backVisionCamera,
            poseOnRobot=Translation3d(0.264, 0.094, 0.58),
            headingOnRobot=Rotation2d(180.0),
            pitchAngleDegrees=0.0
        )

        NamedCommands.registerCommand("Shoot", LaunchSequence(self.fuelSubsystem, self.frontVisionCamera, launchTimeout=5))
        NamedCommands.registerCommand("Intake", Intake(self.fuelSubsystem))
        NamedCommands.registerCommand("Eject", Eject(self.fuelSubsystem))
        NamedCommands.registerCommand("Climb", ClimbUp(self.fuelSubsystem))

        print(f"{self.climbSubsystem.leftArm.get_position().value}")

        # The driver's controller
        self.driverController = commands2.button.CommandXboxController(
            OperatorConstants.DRIVER_CONTROLLER_PORT
        )

        # The operator's controller
        self.operatorController = commands2.button.CommandXboxController(
            OperatorConstants.OPERATOR_CONTROLLER_PORT
        )

        NamedCommands.registerCommand("Aim", Aim(self.driveSubsystem, self.driverController, self.frontVisionCamera))

        # The autonomous chooser
        self.autoChooser = AutoBuilder.buildAutoChooser()

        self.configureBindings()

        # Set the options to show up in the Dashboard for selecting auto modes
        wpilib.SmartDashboard.putData("Auto Chooser", self.autoChooser)

    def configureBindings(self) -> None:
        # While the left bumper on operator controller is held, run the intake command
        # on the fuel subsystem.
        self.operatorController.leftBumper().whileTrue(Intake(self.fuelSubsystem))

        # While the right bumper on the operator controller is held, run the launch
        # sequence command on the fuel subsystem.
        self.operatorController.rightBumper().onTrue(
            LaunchSequence(self.fuelSubsystem, self.frontVisionCamera)
        )

        self.operatorController.x().onTrue(StopLaunch(self.fuelSubsystem))

        # While the A button is held on the operator controller, run the eject command
        # on the fuel subsystem.
        self.operatorController.a().whileTrue(Eject(self.fuelSubsystem))

        # Set the default command for the drive subsystem to the command provided by
        # factory with the values provided by the joystick axes on the driver
        # controller.

        self.driverController.a().whileTrue(Aim(self.driveSubsystem, self.driverController, self.frontVisionCamera))

        self.driveSubsystem.setDefaultCommand(
            Drive(self.driveSubsystem, self.driverController)
        )

        self.driverController.leftBumper().onTrue(ClimbDown(self.climbSubsystem))
        self.driverController.rightBumper().onTrue(ClimbUp(self.climbSubsystem))

        self.fuelSubsystem.run(lambda: self.fuelSubsystem.stop())





    def getAutonomousCommand(self) -> commands2.Command:
        return self.autoChooser.getSelected()
