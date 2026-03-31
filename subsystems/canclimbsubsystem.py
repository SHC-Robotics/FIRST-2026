import commands2
import wpilib

from phoenix6 import hardware, configs, controls, signals
from constants import ClimberConstants


class CANClimbSubsystem(commands2.Subsystem):
    def __init__(self) -> None:
        super().__init__()

        self.leftArm = hardware.TalonFX(ClimberConstants.LEFT_ARM_ID)
        self.rightArm = hardware.TalonFX(ClimberConstants.RIGHT_ARM_ID)

        # Limit switch wired to a DIO port — triggers when arms are fully retracted (down)
        self.limitSwitchLeft = wpilib.DigitalInput(ClimberConstants.LIMIT_SWITCH_LEFT_PORT)
        self.limitSwitchRight = wpilib.DigitalInput(ClimberConstants.LIMIT_SWITCH_RIGHT_PORT)

        # Motion Magic position request (units: rotations, voltage-based)
        # Cruise velocity in the Motion Magic config caps how fast it moves to the target
        # Upgrade path: swap to MotionMagicTorqueCurrentFOC if Phoenix Pro is available
        self.motion_magic_position_request = controls.MotionMagicVoltage(0).with_override_brake_dur_neutral(True)

        config = configs.TalonFXConfiguration()

        # --- Motor output ---
        config.motor_output.neutral_mode = signals.NeutralModeValue.BRAKE
        config.motor_output.inverted = (
            configs.config_groups.InvertedValue.CLOCKWISE_POSITIVE
        )

        # --- Current limits ---
        config.current_limits.supply_current_limit_enable = True
        config.current_limits.supply_current_limit = ClimberConstants.SUPPLY_CURRENT_LIMIT

        # --- Motion Magic profile ---
        # Cruise velocity caps the max speed during the move (rotations/sec)
        # This is how we enforce a safe, slow climb even in position mode
        config.motion_magic.motion_magic_cruise_velocity = ClimberConstants.MM_CRUISE_VELOCITY
        # Acceleration: how quickly it ramps up to cruise velocity (rotations/sec²)
        config.motion_magic.motion_magic_acceleration = ClimberConstants.MM_ACCELERATION
        # Jerk: limits rate of acceleration change for smoother motion (rotations/sec³)
        # Set to 0 to disable jerk limiting
        config.motion_magic.motion_magic_jerk = ClimberConstants.MM_JERK

        # --- Slot 0 PID/feedforward for MotionMagicVoltage ---
        # kS: static friction feedforward (Volts)
        config.slot0.k_s = ClimberConstants.MM_KS
        # kV: velocity feedforward (Volts per rotation/sec)
        config.slot0.k_v = ClimberConstants.MM_KV
        # kA: acceleration feedforward (Volts per rotation/sec²)
        config.slot0.k_a = ClimberConstants.MM_KA
        # kP: proportional gain for position error (Volts per rotation of error)
        config.slot0.k_p = ClimberConstants.MM_KP
        # kI: integral gain
        config.slot0.k_i = ClimberConstants.MM_KI
        # kD: derivative gain
        config.slot0.k_d = ClimberConstants.MM_KD

        self.leftArm.configurator.apply(config)

        config.motor_output.inverted = (
            configs.config_groups.InvertedValue.COUNTER_CLOCKWISE_POSITIVE
        )
        self.rightArm.configurator.apply(config)

        # homePosition is set to 0 after homing completes.
        # Until ClimberHoming runs, this is a best-guess fallback from boot position.
        # Do not rely on this value for accurate positioning until homing has been run.
        self.homePosition = self.leftArm.get_position().value

        self.duty_cycle_request = controls.DutyCycleOut(0)

    def isLeftAtBottom(self) -> bool:
        """Returns True when the left arm limit switch is triggered."""
        return self.limitSwitchLeft.get()

    def isRightAtBottom(self) -> bool:
        """Returns True when the right arm limit switch is triggered."""
        return self.limitSwitchRight.get()

    def isAtBottom(self) -> bool:
        """Returns True when both arm limit switches are triggered."""
        return self.isLeftAtBottom() and self.isRightAtBottom()

    def isLeftAtPosition(self, targetPosition: float, toleranceRotations: float = 0.5) -> bool:
        """Returns True when the left arm is within tolerance of the target position."""
        return abs(self.leftArm.get_position().value - targetPosition) < toleranceRotations

    def isRightAtPosition(self, targetPosition: float, toleranceRotations: float = 0.5) -> bool:
        """Returns True when the right arm is within tolerance of the target position."""
        return abs(self.rightArm.get_position().value - targetPosition) < toleranceRotations

    def isAtPosition(self, targetPosition: float, toleranceRotations: float = 0.5) -> bool:
        """Returns True when both arms are within tolerance of the target position (rotations)."""
        return self.isLeftAtPosition(targetPosition, toleranceRotations) and self.isRightAtPosition(targetPosition, toleranceRotations)

    def setVoltage(self, voltage: float) -> None:
        """Applies direct voltage output. Used for manual control only."""
        self.leftArm.set_control(controls.VoltageOut(voltage))
        #self.leftArm.set_control(self.duty_cycle_request.with_output(voltage))
        self.rightArm.set_control(controls.VoltageOut(voltage))

    def setVoltageLeft(self, voltage: float) -> None:
        self.leftArm.set_control(controls.VoltageOut(voltage))

    def setVoltageRight(self, voltage: float) -> None:
        self.rightArm.set_control(controls.VoltageOut(voltage))

    def setMotionMagicPosition(self, position: float) -> None:
        """
        Commands the arm to a target absolute position (rotations) using Motion Magic.
        Speed is capped by MM_CRUISE_VELOCITY in the motor config.
        position: absolute target in rotations (use initLeftPosition +/- delta)
        """
        self.leftArm.set_control(
            self.motion_magic_position_request.with_position(position)
        )
        self.rightArm.set_control(
            self.motion_magic_position_request.with_position(position)
        )

    def stopLeft(self) -> None:
        self.leftArm.set_control(controls.StaticBrake())

    def stopRight(self) -> None:
        self.rightArm.set_control(controls.StaticBrake())

    def stop(self) -> None:
        self.stopLeft()
        self.stopRight()

    def periodic(self) -> None:
        """Safety: if limit switch is hit, stop the arms regardless of active command."""
        wpilib.SmartDashboard.putNumber("leftArm position", self.leftArm.get_position().value)
        wpilib.SmartDashboard.putBoolean("left switch", self.limitSwitchLeft.get())
        wpilib.SmartDashboard.putBoolean("right switch", self.limitSwitchRight.get())
        # if self.isAtBottom():
        #     self.stop()
