from commands.aim import Aim
from commands.hopper import GrowHopper, JostleHopper, ShrinkHopper
from commands.resetpose import ResetRotation, ZeroRotation
from subsystems.canhoppersubsystem import CANHopperSubsystem
from subsystems.vision_camera import VisionCamera
from subsystems.vision_localizer import VisionLocalizer
import wpilib
import commands2
import commands2.button
import commands2.cmd

from constants import OperatorConstants
from commands.drive import Drive
from commands.eject import Eject
from commands.autolaunch import AutoBackUpLaunch, AutoLaunch
from commands.intake import Intake
from commands.launchsequence import LaunchSequence
from commands.launch import StopLaunch
from subsystems.candrivesubsystem import CANDriveSubsystem
from subsystems.canfuelsubsystem import CANFuelSubsystem

from wpimath.geometry import Translation3d, Rotation2d

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
        self.hopperSubsystem = CANHopperSubsystem()

        self.visionLocalizer = VisionLocalizer(self.driveSubsystem)
        self.frontVisionCamera = VisionCamera("limelight-front")
        self.backVisionCamera = VisionCamera("limelight-back")
        # LL Forward: 0.29, LL Right: 0.11, LL Up: 0.5, LL Yaw: 0
        self.visionLocalizer.addCamera(self.frontVisionCamera)
        # LL Forward: -0.34, LL Right: -0.18, LL Up: 0.51, LL Yaw: 180
        self.visionLocalizer.addCamera(self.backVisionCamera)

        # The autonomous chooser
        self.autoChooser = wpilib.SendableChooser()

        self.configureBindings()

        self.autoChooser.setDefaultOption("None", commands2.cmd.none())
        self.autoChooser.addOption(
            "Shoot in place", AutoLaunch(self.fuelSubsystem),
        )
        self.autoChooser.addOption(
            "Back up shoot", AutoBackUpLaunch(self.driveSubsystem, self.fuelSubsystem),
        )

        # Set the options to show up in the Dashboard for selecting auto modes
        wpilib.SmartDashboard.putData("Auto Chooser", self.autoChooser)

    def configureBindings(self) -> None:
        # While the Y button on the operator controller is held, run the launch
        # sequence command on the fuel subsystem.
        self.operatorController.y().onTrue(
            LaunchSequence(self.fuelSubsystem)
        )

        self.operatorController.x().onTrue(StopLaunch(self.fuelSubsystem))

        # While the left bumper on operator controller is held, run the intake command
        # on the fuel subsystem.
        self.operatorController.leftBumper().whileTrue(Intake(self.fuelSubsystem))

        # While the right bumper is held on the operator controller, run the eject command
        # on the fuel subsystem.
        self.operatorController.rightBumper().whileTrue(Eject(self.fuelSubsystem))

        self.driverController.x().whileTrue(Aim(self.driveSubsystem, self.fuelSubsystem))

        # FOR TESTING PURPOSES ONLY
        self.driverController.leftBumper().onTrue(ShrinkHopper(self.hopperSubsystem))
        self.driverController.rightBumper().onTrue(GrowHopper(self.hopperSubsystem))

        self.driverController.a().whileTrue(JostleHopper(self.hopperSubsystem))

        self.fuelSubsystem.run(lambda: self.fuelSubsystem.stop())

    def getAutonomousCommand(self) -> commands2.Command:
        return self.autoChooser.getSelected()
