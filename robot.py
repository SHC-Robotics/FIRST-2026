import wpilib
import wpilib.drive

JOYSTICK_PORT = 0
LEFT_MOTOR_CHANNEL = 0
RIGHT_MOTOR_CHANNEL = 1


class Robot(wpilib.TimedRobot):
    def robotInit(self):
        leftMotor = wpilib.PWMSparkMax(LEFT_MOTOR_CHANNEL)
        rightMotor = wpilib.PWMSparkMax(RIGHT_MOTOR_CHANNEL)
        self.drive = wpilib.drive.DifferentialDrive(leftMotor, rightMotor)
        self.stick = wpilib.Joystick(JOYSTICK_PORT)

        rightMotor.setInverted(True)

    def teleopPeriodic(self):
        self.drive.arcadeDrive(self.stick.getY(), self.stick.getX())
