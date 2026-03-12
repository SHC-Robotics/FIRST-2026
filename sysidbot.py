import commands2
import wpilib
from commands2 import sysid
from phoenix6.signals import SignalLogger
 
from constants import OperatorConstants
from subsystems.candrivesubsystem import CANDriveSubsystem
from subsystems.canfuelsubsystem import CANFuelSubsystem
 
 
class SysIdRoutineBot:
    def __init__(self) -> None:
        self.driveSubsystem = CANDriveSubsystem()
        self.fuelSubsystem = CANFuelSubsystem()
        self.driverController = commands2.button.CommandXboxController(
            OperatorConstants.DRIVER_CONTROLLER_PORT
        )
        self.configureBindings()
 
    def configureBindings(self) -> None:
        # --- SignalLogger control ---
        # Press Back  → start logging to roboRIO 2 onboard flash (/home/lvuser/logs/)
        # Press Start → stop  logging and flush the .hoot file
        self.driverController.back().onTrue(
            commands2.cmd.runOnce(SignalLogger.start)
        )
        self.driverController.start().onTrue(
            commands2.cmd.runOnce(SignalLogger.stop)
        )
 
        # --- SysId tests (hold button, release to stop) ---
        # A: Quasistatic forward  — slow ramp, characterizes kS + kV → max velocity
        # B: Quasistatic reverse
        # X: Dynamic forward      — step voltage, characterizes kA → moment of inertia
        # Y: Dynamic reverse
        self.driverController.a().whileTrue(
            self.driveSubsystem.sysIdQuasistatic(sysid.SysIdRoutine.Direction.kForward)
        )
        self.driverController.b().whileTrue(
            self.driveSubsystem.sysIdQuasistatic(sysid.SysIdRoutine.Direction.kReverse)
        )
        self.driverController.x().whileTrue(
            self.driveSubsystem.sysIdDynamic(sysid.SysIdRoutine.Direction.kForward)
        )
        self.driverController.y().whileTrue(
            self.driveSubsystem.sysIdDynamic(sysid.SysIdRoutine.Direction.kReverse)
        )
