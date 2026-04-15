from wpimath.geometry import Translation2d

class DriveConstants:
    # Motor controller IDs for drivetrain motors
    LEFT_LEADER_ID = 4
    LEFT_FOLLOWER_ID = 3
    RIGHT_LEADER_ID = 2
    RIGHT_FOLLOWER_ID = 1

    DRIVE_SPEED_MULT = 0.6

    DRIVE_SPEED_MULT = 0.6

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

    INITIAL_SHOOT_MULT = 0.65
    SHOOT_MULT_LOWER = 0.5
    SHOOT_MULT_UPPER = 0.85

    # Voltage values for various fuel operations. These values may need to be tuned
    # based on exact robot construction.
    # See the Software Guide for tuning information
    INTAKING_FEEDER_VOLTAGE = -10.0
    INTAKING_INTAKE_VOLTAGE = 4.0
    LAUNCHING_FEEDER_VOLTAGE = 9.0
    LAUNCHING_LAUNCHER_VOLTAGE = 12.0
    SPIN_UP_FEEDER_VOLTAGE = -6.0 
    SPIN_UP_SECONDS = 1.0

    AUTO_SHOOTING_MULTIPLIER = 0.6


class OperatorConstants:
    # Port constants for driver and operator controllers. These should match the
    # values in the Joystick tab of the Driver Station software
    DRIVER_CONTROLLER_PORT = 0
    OPERATOR_CONTROLLER_PORT = 1

    # This value is multiplied by the joystick value when rotating the robot to
    # help avoid turning too fast and being difficult to control
    DRIVE_SCALING = 0.5
    ROTATION_SCALING = 0.5


class AprilTagIds:
    RED_HUB_TAGS = [8,5,9,10,11,2]
    BLUE_HUB_TAGS = [18,27,26,25,21,24]

    RED_HUB_CENTER = 10
    BLUE_HUB_CENTER = 26


class FieldConstants:
    RED_HUB_POSITION = Translation2d(11.915394, 4.03479) # 469.11 in, 158.85 in
    BLUE_HUB_POSITION = Translation2d(4.625594, 4.03479) # 182.11 in, 158.85 in


class HopperConstants:
    EXTENSION_MOTOR_ID = 7
    EXTENSION_MOTOR_CURRENT_LIMIT = 60
