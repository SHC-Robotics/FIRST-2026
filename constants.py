class DriveConstants:
    # Motor controller IDs for drivetrain motors
    LEFT_LEADER_ID = 4
    LEFT_FOLLOWER_ID = 3
    RIGHT_LEADER_ID = 2
    RIGHT_FOLLOWER_ID = 1

    # Current limit for drivetrain motors. 60A is a reasonable maximum to reduce
    # likelihood of tripping breakers or damaging CIM motors
    DRIVE_MOTOR_CURRENT_LIMIT = 60


class FuelConstants:
    # Motor controller IDs for Fuel Mechanism motors
    FEEDER_MOTOR_ID = 6
    INTAKE_LAUNCHER_MOTOR_ID = 5

    # Current limit and nominal voltage for fuel mechanism motors.
    FEEDER_MOTOR_CURRENT_LIMIT = 60
    LAUNCHER_MOTOR_CURRENT_LIMIT = 60

    # Voltage values for various fuel operations. These values may need to be tuned
    # based on exact robot construction.
    # See the Software Guide for tuning information
    INTAKING_FEEDER_VOLTAGE = -12.0 
    INTAKING_INTAKE_VOLTAGE = 8.0 
    LAUNCHING_FEEDER_VOLTAGE = 9.0 
    LAUNCHING_LAUNCHER_VOLTAGE = 12.0
    SPIN_UP_FEEDER_VOLTAGE = -6.0 
    SPIN_UP_SECONDS = 1.0


class OperatorConstants:
    # Port constants for driver and operator controllers. These should match the
    # values in the Joystick tab of the Driver Station software
    DRIVER_CONTROLLER_PORT = 0
    OPERATOR_CONTROLLER_PORT = 1

    # This value is multiplied by the joystick value when rotating the robot to
    # help avoid turning too fast and being difficult to control
    DRIVE_SCALING = 0.5
    ROTATION_SCALING = 0.5


class ClimberConstants:
    # CAN IDs
    LEFT_ARM_ID = 7
    RIGHT_ARM_ID = 8

    # DIO port for the limit switch (triggered when arms are fully retracted/down)
    # TODO: set to the actual DIO channel the limit switch is wired to
    LIMIT_SWITCH_PORT = 0

    # --- Position targets (rotations) ---
    # TODO: verify CLIMB_UP_DELTA by manually driving the arm to full extension and reading
    # the encoder value in Phoenix Tuner X or via print statements
    CLIMB_UP_DELTA = 24      # Rotations from home = arms fully extended (up)

    # Per-loop step for homing routine (rotations per 20ms loop cycle)
    HOMING_STEP = 0.02

    # How close to the target counts as "arrived" (rotations)
    POSITION_TOLERANCE = 0.5

    # --- Motion Magic profile ---
    # Cruise velocity caps the max speed during any position move — keep this low for safety
    # TODO: tune on robot; 2 rot/s is a conservative starting point
    MM_CRUISE_VELOCITY = 0.5   # rotations/sec
    MM_ACCELERATION = 1.0      # rotations/sec² — reaches cruise in ~0.5s at this value
    MM_JERK = 0                # rotations/sec³ — 0 disables jerk limiting

    # --- Current limit (Amps) ---
    # Supply current limit protects the motor and breaker during high-load climb
    # TODO: tune based on your gearbox and robot weight; 40-60A is a reasonable starting point
    SUPPLY_CURRENT_LIMIT = 60

    # --- Slot 0 PID/feedforward gains for MotionMagicVoltage ---
    # Recommended tuning order: kS → kV (via Phoenix Tuner X SysId) → kP (manual sweep)
    # kP responds to position error (Volts per rotation of error)
    MM_KS = 0.0   # Static friction feedforward (Volts) — TODO: run SysId or start at 0
    MM_KV = 0.0   # Velocity feedforward (Volts per rot/s) — TODO: run SysId
    MM_KA = 0.0   # Acceleration feedforward — usually small, start at 0
    MM_KP = 5.0   # Proportional gain — TODO: increase until arm reaches position without oscillating
    MM_KI = 0.0   # Integral gain — leave at 0
    MM_KD = 0.0   # Derivative gain — leave at 0