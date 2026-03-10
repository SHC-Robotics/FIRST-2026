from subsystems.vision_camera import VisionCamera
from subsystems.vision_localizer import VisionLocalizer
import wpilib
import commands2
import commands2.button
import commands2.cmd

from constants import OperatorConstants
from commands.drive import Drive
from commands.eject import Eject
from commands.exampleauto import ExampleAuto
from commands.intake import Intake
from commands.climb_down import ClimbDown, ClimbDownManual
from commands.climb_up import ClimbUp, ClimbUpManual
from commands.homing import ClimberHoming
from commands.launchsequence import LaunchSequence
from subsystems.candrivesubsystem import CANDriveSubsystem
from subsystems.canfuelsubsystem import CANFuelSubsystem
from subsystems.canclimbsubsystem import CANClimbSubsystem

from wpimath.geometry import Translation3d, Rotation2d
from pathplannerlib.auto import AutoBuilder, NamedCommands

from pathplannerlib.auto import AutoBuilder


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
        self.visionCamera = VisionCamera("limelight-front")
        self.visionLocalizer.addCamera(
            self.visionCamera,
            poseOnRobot=Translation3d(), # TODO: measure this
            headingOnRobot=Rotation2d(), # TODO: measure this
            pitchAngleDegrees=0.0,
        )

        NamedCommands.registerCommand("Shoot", LaunchSequence(self.fuelSubsystem))
        NamedCommands.registerCommand("Intake", Intake(self.fuelSubsystem))
        NamedCommands.registerCommand("Eject", Intake(self.fuelSubsystem))
        NamedCommands.registerCommand("Climb", ClimbUp(self.climbSubsystem))  # fixed: was fuelSubsystem

        # Run the homing routine immediately on startup to zero the climber encoders
        #ClimberHoming(self.climbSubsystem).schedule()

        # The driver's controller
        self.driverController = commands2.button.CommandXboxController(
            OperatorConstants.DRIVER_CONTROLLER_PORT
        )

        # The operator's controller
        self.operatorController = commands2.button.CommandXboxController(
            OperatorConstants.OPERATOR_CONTROLLER_PORT
        )

        # The autonomous chooser
        self.autoChooser = AutoBuilder.buildAutoChooser()

        self.configureBindings()

<<<<<<< Updated upstream
        # Set the options to show up in the Dashboard for selecting auto modes
        wpilib.SmartDashboard.putData("Auto Chooser", self.autoChooser)
=======
        # Set the options to show up in the Dashboard for selecting auto modes.
        self.autoChooser.setDefaultOption(
            "Autonomous", ExampleAuto(self.driveSubsystem, self.fuelSubsystem)
        )
        wpilib.SmartDashboard.putData("Autonomous", self.autoChooser)
        
        pf = wpilib.PortForwarder.getInstance()

        # Limelight 1 (ID 0) - Mapped to 5801-5809
        # Maps local port 580x to Limelight IP 172.29.0.1 on port 580x
        for port in range(5801, 5810):
            pf.add(port, "172.29.0.1", port)
>>>>>>> Stashed changes

    def configureBindings(self) -> None:
        # While the left bumper on operator controller is held, run the intake command
        # on the fuel subsystem.
        self.operatorController.leftBumper().whileTrue(Intake(self.fuelSubsystem))

        # While the right bumper on the operator controller is held, run the launch
        # sequence command on the fuel subsystem.
        self.operatorController.rightBumper().whileTrue(
            LaunchSequence(self.fuelSubsystem)
        )

        # While the A button is held on the operator controller, run the eject command
        # on the fuel subsystem.
        self.operatorController.a().whileTrue(Eject(self.fuelSubsystem))

        # Set the default command for the drive subsystem to the command provided by
        # factory with the values provided by the joystick axes on the driver
        # controller.
        self.driveSubsystem.setDefaultCommand(
            Drive(self.driveSubsystem, self.driverController)
        )

        #self.driverController.leftBumper().onTrue(ClimbDown(self.climbSubsystem))
        #self.driverController.rightBumper().onTrue(ClimbUp(self.climbSubsystem))

        self.driverController.x().whileTrue(ClimbUpManual(self.climbSubsystem))
        self.driverController.b().whileTrue(ClimbDownManual(self.climbSubsystem))

        self.fuelSubsystem.run(lambda: self.fuelSubsystem.stop())

    def getAutonomousCommand(self) -> commands2.Command:
        return self.autoChooser.getSelected()
