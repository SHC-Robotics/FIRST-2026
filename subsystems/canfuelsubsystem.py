import commands2
import rev
import wpilib

from constants import FuelConstants


class CANFuelSubsystem(commands2.Subsystem):
    def __init__(self) -> None:
        super().__init__()

        # Instantiate each of the motors on the launcher mechanism
        self.intakeLauncherRoller = rev.SparkMax(
            FuelConstants.INTAKE_LAUNCHER_MOTOR_ID,
            rev.SparkLowLevel.MotorType.kBrushed,
        )
        self.feederRoller = rev.SparkMax(
            FuelConstants.FEEDER_MOTOR_ID, rev.SparkLowLevel.MotorType.kBrushed
        )

        # Put default values for various fuel operations onto the dashboard.
        # All commands using this subsystem pull values from the dashboard to allow
        # values to be tuned easily. Values in constants.py must be replaced for changes to persist.
        wpilib.SmartDashboard.putNumber(
            "Intaking feeder roller value", FuelConstants.INTAKING_FEEDER_VOLTAGE
        )
        wpilib.SmartDashboard.putNumber(
            "Intaking intake roller value", FuelConstants.INTAKING_INTAKE_VOLTAGE
        )
        wpilib.SmartDashboard.putNumber(
            "Launching feeder roller value", FuelConstants.LAUNCHING_FEEDER_VOLTAGE
        )
        wpilib.SmartDashboard.putNumber(
            "Launching launcher roller value", FuelConstants.LAUNCHING_LAUNCHER_VOLTAGE
        )
        wpilib.SmartDashboard.putNumber(
            "Spin-up feeder roller value", FuelConstants.SPIN_UP_FEEDER_VOLTAGE
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
        launcherConfig.inverted(True)
        launcherConfig.smartCurrentLimit(FuelConstants.LAUNCHER_MOTOR_CURRENT_LIMIT)
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
