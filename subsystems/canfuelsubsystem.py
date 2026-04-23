import commands2
import rev
import wpilib
from wpimath.controller import SimpleMotorFeedforwardRadians, PIDController

from constants import FuelConstants

RPM_TO_RPS = 1 / 60


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

        self.intakeLauncherFeedforward = SimpleMotorFeedforwardRadians(0, 0, 0) # kS, kV - volts/radians/s, kA
        self.feederFeedforward = SimpleMotorFeedforwardRadians(0, 0, 0)

        self.intakeLauncherPid = PIDController(0, 0, 0) # kp, ki, kd
        self.feederPid = PIDController(0, 0, 0)

    # A method to set the voltage of the intake roller
    def setIntakeLauncherRoller(self, rps: float) -> None:
        self.intakeLauncherRoller.set(
            self.intakeLauncherFeedforward.calculate(rps)
                + self.intakeLauncherPid.calculate(
                    self.intakeLauncherEncoder.getVelocity() * RPM_TO_RPS, rps))

    # A method to set the voltage of the feeder roller
    def setFeederRoller(self, rps: float) -> None:
        self.feederRoller.set(
            self.feederFeedforward.calculate(rps)
                + self.feederPid.calculate(
                    self.feederEncoder.getVelocity() * RPM_TO_RPS, rps))

    # A method to stop the rollers
    def stop(self) -> None:
        self.feederRoller.set(0)
        self.intakeLauncherRoller.set(0)

    def periodic(self) -> None:
        leftTrigger = self.controller.getLeftTriggerAxis()
        rightTrigger = self.controller.getRightTriggerAxis()

        if leftTrigger < 0.1:
            leftTrigger = 0

        if rightTrigger < 0.1:
            rightTrigger = 0

        self.multiplier -= leftTrigger * 0.001
        self.multiplier += rightTrigger * 0.001

        self.multiplier = max(min(self.multiplier, FuelConstants.SHOOT_MULT_UPPER), FuelConstants.SHOOT_MULT_LOWER)

        wpilib.SmartDashboard.putNumber("Shooting multiplier", self.multiplier)
        wpilib.SmartDashboard.putNumber("Intake launcher rps", self.intakeLauncherEncoder.getVelocity() * RPM_TO_RPS)
        wpilib.SmartDashboard.putNumber("Feeder rps", self.feederEncoder.getVelocity() * RPM_TO_RPS)
