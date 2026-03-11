from constants import OperatorConstants
from subsystems.candrivesubsystem import CANDriveSubsystem
from subsystems.canfuelsubsystem import CANFuelSubsystem
import commands2
import wpilib


class SysIdRoutineBot:
    def __init__(self) -> None:
        self.driveSubsystem = CANDriveSubsystem()
        self.fuelSubsystem = CANFuelSubsystem()

        self.driverController = commands2.button.CommandXboxController(OperatorConstants.DRIVER_CONTROLLER_PORT)

        self.configureBindings()

    def configureBindings(self) -> None:
        self.driverController.a().whileTrue(self.driveSubsystem.sysIdQuasistatic(commands2.sysid.SysIdRoutine.Direction.kForward))
        self.driverController.b().whileTrue(self.driveSubsystem.sysIdQuasistatic(commands2.sysid.SysIdRoutine.Direction.kReverse))
        self.driverController.x().whileTrue(self.driveSubsystem.sysIdDynamic(commands2.sysid.SysIdRoutine.Direction.kForward))
        self.driverController.y().whileTrue(self.driveSubsystem.sysIdDynamic(commands2.sysid.SysIdRoutine.Direction.kReverse))
