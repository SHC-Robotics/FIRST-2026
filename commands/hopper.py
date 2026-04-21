from commands.intake import Intake
from commands.launch import Launch
from commands.spinup import SpinUp
import commands2
from subsystems.canfuelsubsystem import CANFuelSubsystem
import wpilib

from constants import FuelConstants, HopperConstants
from subsystems.canhoppersubsystem import CANHopperSubsystem


class GrowHopper(commands2.Command):
    def __init__(self, hopperSubsystem: CANHopperSubsystem) -> None:
        super().__init__()

        self.hopperSubsystem = hopperSubsystem
        self.addRequirements(self.hopperSubsystem)

    def initialize(self) -> None:
        if self.hopperSubsystem.getExtensionPosition() > HopperConstants.EXTENSION_NUM_ROTATIONS:
            return

        self.hopperSubsystem.setExtension(HopperConstants.EXTENSION_MOTOR_VOLTAGE)

    def end(self, interrupted: bool) -> None:
        self.hopperSubsystem.stop()

    def isFinished(self) -> bool:
        wpilib.SmartDashboard.putNumber("Hopper position", self.hopperSubsystem.getExtensionPosition())

        if self.hopperSubsystem.getExtensionPosition() > HopperConstants.EXTENSION_NUM_ROTATIONS:
            return True

        return False

class ShrinkHopper(commands2.Command):
    def __init__(self, hopperSubsystem: CANHopperSubsystem) -> None:
        super().__init__()

        self.hopperSubsystem = hopperSubsystem
        self.addRequirements(self.hopperSubsystem)

    def initialize(self) -> None:
        if self.hopperSubsystem.getExtensionPosition() < 0.1:
            return

        self.hopperSubsystem.setExtension(-HopperConstants.EXTENSION_MOTOR_VOLTAGE)

    def end(self, interrupted: bool) -> None:
        self.hopperSubsystem.stop()

    def isFinished(self) -> bool:
        wpilib.SmartDashboard.putNumber("Hopper position", self.hopperSubsystem.getExtensionPosition())

        if self.hopperSubsystem.getExtensionPosition() < 0.1:
            return True

        return False

class JostleHopper(commands2.Command):
    def __init__(self, hopperSubsystem: CANHopperSubsystem) -> None:
        super().__init__()

        self.hopperSubsystem = hopperSubsystem
        self.lastTime = 0
        self.direction = 1
        self.addRequirements(self.hopperSubsystem)

    def initialize(self) -> None:
        self.lastTime = wpilib.Timer.getFPGATimestamp()
        self.direction = 1

    def execute(self) -> None:
        now = wpilib.Timer.getFPGATimestamp()
        if now - self.lastTime > HopperConstants.JOSTLE_RATE_SECONDS:
            self.lastTime = now
            self.hopperSubsystem.setExtension(self.direction * HopperConstants.EXTENSION_MOTOR_VOLTAGE)
            self.direction = -self.direction

    def end(self, interrupted: bool) -> None:
        self.hopperSubsystem.stop()

    def isFinished(self) -> bool:
        return False

class ShrinkThenJostleHopper(commands2.SequentialCommandGroup):
    def __init__(self, hopperSubsystem: CANHopperSubsystem) -> None:
        super().__init__()

        self.addCommands(
            ShrinkHopper(hopperSubsystem),
            JostleHopper(hopperSubsystem),
        )

class IntakeGrowHopper(commands2.ParallelCommandGroup):
    def __init__(self, fuelSubsystem: CANFuelSubsystem, hopperSubsystem: CANHopperSubsystem) -> None:
        super().__init__()

        self.addCommands(
            GrowHopper(hopperSubsystem),
            Intake(fuelSubsystem)
        )

class LaunchShrinkHopper(commands2.ParallelCommandGroup):
    def __init__(self, fuelSubsystem: CANFuelSubsystem, hopperSubsystem: CANHopperSubsystem) -> None:
        super().__init__()

        self.addCommands(
            ShrinkHopper(hopperSubsystem),
            Launch(fuelSubsystem)
        )

class LaunchSequenceShrinkHopper(commands2.SequentialCommandGroup):
    def __init__(self, fuelSubsystem: CANFuelSubsystem, hopperSubsystem: CANHopperSubsystem, launchTimeout=None) -> None:
        super().__init__()

        if launchTimeout:
            self.addCommands(
                SpinUp(fuelSubsystem).withTimeout(FuelConstants.SPIN_UP_SECONDS),
                LaunchShrinkHopper(fuelSubsystem, hopperSubsystem).withTimeout(launchTimeout),
            )
        else: 
            self.addCommands(
                SpinUp(fuelSubsystem).withTimeout(FuelConstants.SPIN_UP_SECONDS),
                LaunchShrinkHopper(fuelSubsystem, hopperSubsystem),
            )
