import wpilib
import wpilib.drive


class Robot(wpilib.TimedRobot):
    def robotInit(self):
        leftMotor = wpilib.PWMSparkMax(0)
        rightMotor = wpilib.PWMSparkMax(1)
        self.robotDrive = wpilib.drive.DifferentialDrive(leftMotor, rightMotor)
        self.stick = wpilib.Joystick(0)

        rightMotor.setInverted(True)

    def teleopPeriodic(self):
        self.robotDrive.arcadeDrive(self.stick.getY(), self.stick.getX())
