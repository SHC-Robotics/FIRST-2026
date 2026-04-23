import commands2
import rev
import wpilib

from constants import FuelConstants


class CANFuelSubsystem(commands2.Subsystem):
    def __init__(self, operatorController) -> None:
        super().__init__()


        #current distance from target
        self.distance = None

        self.multiplier = FuelConstants.INITIAL_SHOOT_MULT

        self.controller = operatorController

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
        feederConfig.inverted(True)
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
        self.intakeLauncherRoller.configure(
            launcherConfig,
            rev.ResetMode.kResetSafeParameters,
            rev.PersistMode.kPersistParameters,
        )

        self.intakeLauncherEncoder = self.intakeLauncherRoller.getEncoder()
        self.feederEncoder = self.feederRoller.getEncoder()

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

    def periodic(self) -> None:
        rightY = self.controller.getRightY()
        if rightY < 0.1 and rightY > -0.1:
            rightY = 0

        self.multiplier -= rightY * 0.002

        leftTrigger = self.controller.leftTrigger()
        rightTrigger = self.controller.rightTrigger()

        if leftTrigger:
            self.multiplier = 0.64

        if rightTrigger:
            self.multiplier = 0.85

        self.multiplier = max(min(self.multiplier, FuelConstants.SHOOT_MULT_UPPER), FuelConstants.SHOOT_MULT_LOWER)

        wpilib.SmartDashboard.putNumber("Shooting multiplier", self.multiplier)
        wpilib.SmartDashboard.putNumber("Intake launcher rpm", self.intakeLauncherEncoder.getVelocity())
        wpilib.SmartDashboard.putNumber("Feeder rpm", self.feederEncoder.getVelocity())
