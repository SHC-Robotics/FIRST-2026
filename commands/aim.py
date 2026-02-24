import commands2
import wpilib

from constants import FuelConstants
from subsystems.candrivesubsystem import CANDriveSubsystem

ALIGNED_THRESHOLD = 0.05


class Aim(commands2.Command):
    """
    A Command that represents the complete action of launching fuel.
    Requires the fuel subsystem.
    """

    def __init__(self, driveSubsystem: CANDriveSubsystem, driverController, visionCamera) -> None:
        super().__init__()

        self.driveSubsystem = driveSubsystem
        self.controller = driverController
        self.camera = visionCamera
        self.addRequirements(self.driveSubsystem)  



    def initialize(self) -> None:
        #get current robotpose from the drive subsystem
        #curentPose = self.driveSubsystem.getPose()

        #should check if primary tag id is in tagList and to turn towards it if it is
        #2 ids so might want to average data between the two to get dead center
        #https://docs.limelightvision.io/docs/docs-limelight/apis/complete-networktables-api

        #can set priorityid to ignore other tags if there's any benefit towards aiming

        #dif tags for dif alliance sides
        self.tagList = []

        # Red alliance shooter tags
        redTags = [9, 10]
        # Blue alliance shooter tags
        blueTags = [26, 25]

        # Define target tags depending on alliance
        if wpilib.DriverStation.getAlliance() == wpilib.DriverStation.Alliance.kRed:
            self.tagList = redTags
        else:
            self.tagList = blueTags

        raw_tags = self.camera.getRawFiducials()
        max_area = -1.0
        best_id = None

        # Use biggest area as main target
        for i in range(0, len(raw_tags), 7):
            tag_id = int(raw_tags[i])
            area = raw_tags[i+3]

            if tag_id in self.tagList:
                if area > max_area:
                    max_area = area
                    best_id = tag_id

        self.mainTarget = best_id

        print(f"started aiming at tag {self.mainTarget}")

    def execute(self) -> None:
        #grab raw tag id from camera
        raw_tags = self.camera.getRawFiducials()

        tx = None
        area = None
        dist = None

        for i in range(0, len(raw_tags), 7):
            tag_id = int(raw_tags[i])
            if tag_id == self.mainTarget:
                tx = raw_tags[i+1]
                area = raw_tags[i+3]
                dist = raw_tags[i+4]
                break

        if not tx:
            self.driveSubsystem.driveArcade(0, 0)
            return

        # need to check wpilib's orientationController idk if this works
        rotation_output = self.driveSubsystem.orientationController.calculate(tx, 0) 
        rotation_output = max(min(rotation_output, 0.15), -0.15)
        self.driveSubsystem.driveArcade(0, rotation_output)

    def isFinished(self) -> bool:
        # Finished if tag goes outside camera FOV or robot is lined up
        raw_tags = self.camera.getRawFiducials()

        tx = None
        area = None
        dist = None

        for i in range(0, len(raw_tags), 7):
            tag_id = int(raw_tags[i])
            if tag_id == self.mainTarget:
                tx = raw_tags[i+1]
                area = raw_tags[i+3]
                dist = raw_tags[i+4]
                break

        if not tx:
            return True

        if tx < ALIGNED_THRESHOLD and tx > -ALIGNED_THRESHOLD:
            return True

        return False
    
    def end(self, interrupted: bool):
        # Stop motors
        self.driveSubsystem.driveArcade(0, 0)
        print(f"finished aiming at {self.mainTarget}")
