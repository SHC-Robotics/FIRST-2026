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
from commands.launchsequence import LaunchSequence
from subsystems.candrivesubsystem import CANDriveSubsystem
from subsystems.canfuelsubsystem import CANFuelSubsystem

from wpimath.geometry import Translation3d, Rotation2d
from pathplannerlib.auto import AutoBuilder, NamedCommands


class RobotContainer:
    """
    The robot container, which stores the robot's subsystems, controllers, binds
    buttons to commands, and manages autonomous modes.
    """

    def __init__(self) -> None:
        # Subsystems
        self.driveSubsystem = CANDriveSubsystem()
        self.fuelSubsystem  = CANFuelSubsystem()

        self.visionLocalizer    = VisionLocalizer(self.driveSubsystem)
        self.frontVisionCamera  = VisionCamera("limelight-front")
        self.backVisionCamera   = VisionCamera("limelight-back")

        self.visionLocalizer.addCamera(
            self.frontVisionCamera,
            poseOnRobot=Translation3d(0.3066, 0.1056, 0.66),  # Forward: +X, Right: +Y, Up: +Z
            headingOnRobot=Rotation2d.fromDegrees(0.0),         # front-facing
            pitchAngleDegrees=0.0,
        )
        self.visionLocalizer.addCamera(
            self.backVisionCamera,
            poseOnRobot=Translation3d(0.264, 0.094, 0.58),
            headingOnRobot=Rotation2d.fromDegrees(180.0),        # rear-facing — was Rotation2d(180.0) which is radians
            pitchAngleDegrees=0.0,
        )

        # Named commands for PathPlanner autos
        NamedCommands.registerCommand("Shoot",  LaunchSequence(self.fuelSubsystem, self.frontVisionCamera, launchTimeout=5))
        NamedCommands.registerCommand("Intake", Intake(self.fuelSubsystem))
        NamedCommands.registerCommand("Eject",  Eject(self.fuelSubsystem))
        NamedCommands.registerCommand("Aim",    Aim(self.driveSubsystem, None, self.frontVisionCamera))

        # Controllers
        self.driverController   = commands2.button.CommandXboxController(OperatorConstants.DRIVER_CONTROLLER_PORT)
        self.operatorController = commands2.button.CommandXboxController(OperatorConstants.OPERATOR_CONTROLLER_PORT)

        # Auto chooser — must be after AutoBuilder.configure() in CANDriveSubsystem
        self.autoChooser = AutoBuilder.buildAutoChooser()
        wpilib.SmartDashboard.putData("Auto Chooser", self.autoChooser)

        self.configureBindings()

    def configureBindings(self) -> None:
        # Operator controls
        self.operatorController.leftBumper().whileTrue(Intake(self.fuelSubsystem))
        self.operatorController.rightBumper().onTrue(
            LaunchSequence(self.fuelSubsystem, self.frontVisionCamera)
        )
        self.operatorController.a().whileTrue(Eject(self.fuelSubsystem))

        # Driver controls
        self.driverController.a().whileTrue(
            Aim(self.driveSubsystem, self.driverController, self.frontVisionCamera)
        )

        # Default drive command
        self.driveSubsystem.setDefaultCommand(
            Drive(self.driveSubsystem, self.driverController)
        )

        self.fuelSubsystem.run(lambda: self.fuelSubsystem.stop())

    def getAutonomousCommand(self) -> commands2.Command:
        return self.autoChooser.getSelected()