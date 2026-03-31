from commands.aim import Aim
from commands.climb_lift import ClimbLift
from commands.resetpose import ResetRotation
from subsystems.vision_camera import VisionCamera
from subsystems.vision_localizer import VisionLocalizer
import wpilib
import commands2
import commands2.button
import commands2.cmd

from constants import OperatorConstants
from commands.drive import Drive
from commands.eject import Eject
from commands.intake import Intake
from commands.climb_down import ClimbDown
from commands.climb_up import ClimbUp
from commands.climb_manual import ClimbManual
from commands.homing import ClimbHomeManual, ClimberHoming
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
        # The driver's controller
        self.driverController = commands2.button.CommandXboxController(
            OperatorConstants.DRIVER_CONTROLLER_PORT
        )

        # The operator's controller
        self.operatorController = commands2.button.CommandXboxController(
            OperatorConstants.OPERATOR_CONTROLLER_PORT
        )

        # The robot's subsystems.
        # A Subsystem is a collection of motors, sensors, and other hardware objects that are operated on by a Command.
        self.driveSubsystem = CANDriveSubsystem()
        self.fuelSubsystem = CANFuelSubsystem(self.operatorController)
        self.climbSubsystem = CANClimbSubsystem()

        self.visionLocalizer = VisionLocalizer(self.driveSubsystem)
        self.frontVisionCamera = VisionCamera("limelight-front")
        self.backVisionCamera = VisionCamera("limelight-back")
        # LL Forward: 0.29, LL Right: 0.11, LL Up: 0.5, LL Yaw: 0
        self.visionLocalizer.addCamera(self.frontVisionCamera)
        # LL Forward: -0.34, LL Right: -0.18, LL Up: 0.51, LL Yaw: 180
        self.visionLocalizer.addCamera(self.backVisionCamera)

        NamedCommands.registerCommand("Shoot", LaunchSequence(self.fuelSubsystem, launchTimeout=5))
        NamedCommands.registerCommand("Intake", Intake(self.fuelSubsystem))
        NamedCommands.registerCommand("Eject", Eject(self.fuelSubsystem))
        NamedCommands.registerCommand("ClimbUp", ClimbUp(self.climbSubsystem))
        NamedCommands.registerCommand("ClimbLift", ClimbLift(self.climbSubsystem))
        NamedCommands.registerCommand("Aim", Aim(self.driveSubsystem))

        # The autonomous chooser
        self.autoChooser = AutoBuilder.buildAutoChooser()

        self.configureBindings()

        # Set the options to show up in the Dashboard for selecting auto modes
        wpilib.SmartDashboard.putData("Auto Chooser", self.autoChooser)

    def configureBindings(self) -> None:
        # While the Y button on the operator controller is held, run the launch
        # sequence command on the fuel subsystem.
        self.operatorController.y().onTrue(
            LaunchSequence(self.fuelSubsystem)
        )

        self.operatorController.x().onTrue(StopLaunch(self.fuelSubsystem))

        self.operatorController.a().onTrue(ClimbHomeManual(self.climbSubsystem))

        # While the left bumper on operator controller is held, run the intake command
        # on the fuel subsystem.
        self.operatorController.leftBumper().whileTrue(Intake(self.fuelSubsystem))

        # While the right bumper is held on the operator controller, run the eject command
        # on the fuel subsystem.
        self.operatorController.rightBumper().whileTrue(Eject(self.fuelSubsystem))

        self.driverController.leftBumper().onTrue(ClimbLift(self.climbSubsystem))
        self.driverController.rightBumper().onTrue(ClimbUp(self.climbSubsystem))

        # Set the default command for the drive subsystem to the command provided by
        # factory with the values provided by the joystick axes on the driver
        # controller.

        self.driveSubsystem.setDefaultCommand(
            Drive(self.driveSubsystem, self.driverController)
        )

        self.climbSubsystem.setDefaultCommand(
            ClimbManual(self.climbSubsystem, self.operatorController)
        )

        self.driverController.a().whileTrue(ResetRotation(self.driveSubsystem, Rotation2d.fromDegrees(0)))

        self.driverController.x().whileTrue(Aim(self.driveSubsystem))

        self.fuelSubsystem.run(lambda: self.fuelSubsystem.stop())

    def getAutonomousCommand(self) -> commands2.Command:
        return self.autoChooser.getSelected()
