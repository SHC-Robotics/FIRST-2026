import commands2
import wpilib

from constants import FuelConstants
from subsystems.candrivesubsystem import CANDriveSubsystem


class Aim(commands2.Command):
    """
    A Command that represents the complete action of launching fuel.
    Requires the fuel subsystem.
    """

    def __init__(self, driveSubsystem: CANDriveSubsystem, driverController) -> None:
        super().__init__()

        self.driveSubsystem = driveSubsystem
        self.controller = driverController
        self.addRequirements(self.driveSubsystem)

        #ids of hub april tags
        self.tagList = [9, 10]


        #tester for now; need to get target_degrees from checking april tag yaw and pitch
        self.target_degrees = 180



    def initialize(self) -> None:
        #get current robotpose from the drive subsystem
        #curentPose = self.driveSubsystem.getPose()

        #should check if primary tag id is in tagList and to turn towards it if it is
        #2 ids so might want to average data between the two to get dead center
        #https://docs.limelightvision.io/docs/docs-limelight/apis/complete-networktables-api

        #can set priorityid to ignore other tags if there's any benefit towards aiming
        pass

    def execute(self) -> None:
        #turns robot towards target degree of rotation
        self.driveSubsystem.driveToOrientation(self.target_degrees, xSpeed = 0)



    def isFinished(self) -> bool:
        #check if finished
        return self.driveSubsystem.isAtTargetOrientation()
    
    def end(self, interrupted: bool):
        #stop motors
        self.drive.DriveArcade(0, 0)