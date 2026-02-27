import commands2
import rev
import wpilib

from constants import FuelConstants


class CANFuelSubsystem(commands2.Subsystem):
    def __init__(self) -> None:
        super().__init__()


        #current distance from target
        self.distance = None

        # Instantiate each of the motors on the launcher mechanism
        self.intakeLauncherRoller = rev.SparkMax(
            FuelConstants.INTAKE_LAUNCHER_MOTOR_ID,
            rev.SparkLowLevel.MotorType.kBrushless,
        )
        self.feederRoller = rev.SparkMax(
            FuelConstants.FEEDER_MOTOR_ID, 
            rev.SparkLowLevel.MotorType.kBrushless
        )


        # Create the configuration for the feeder roller, set a current limit and
        # apply the config to the controller
        feederConfig = rev.SparkMaxConfig()
        feederConfig.smartCurrentLimit(FuelConstants.FEEDER_MOTOR_CURRENT_LIMIT)
        self.feederRoller.configure(
            feederConfig,
            rev.ResetMode.kResetSafeParameters,
            rev.PersistMode.kPersistParameters,
        )

        # Create the configuration for the launcher roller, set a current limit,
        # set the motor to inverted so that positive values are used for both
        # intaking and launching, and apply the config to the controller
        launcherConfig = rev.SparkMaxConfig()
        launcherConfig.smartCurrentLimit(FuelConstants.LAUNCHER_MOTOR_CURRENT_LIMIT)
        launcherConfig.inverted(True)
        self.intakeLauncherRoller.configure(
            launcherConfig,
            rev.ResetMode.kResetSafeParameters,
            rev.PersistMode.kPersistParameters,
        )

    # A method to set the voltage of the intake roller
    def setIntakeLauncherRoller(self, voltage: float) -> None:
        self.intakeLauncherRoller.setVoltage(voltage)

    # A method to set the voltage of the feeder roller
    def setFeederRoller(self, voltage: float) -> None:
        self.feederRoller.setVoltage(voltage)

    # A method to stop the rollers
    def stop(self) -> None:
        self.feederRoller.set(0)
        self.intakeLauncherRoller.set(0)
