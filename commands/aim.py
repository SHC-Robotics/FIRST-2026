import commands2
import wpilib

from constants import FuelConstants
from subsystems.candrivesubsystem import CANDriveSubsystem


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

        #red alliance shooter tags
        redTags = [9, 10]
        #blue alliance shooter tags
        blueTags = [26, 25]

        #define target tags depending on alliance
        # if wpilib.DriverStation.getAlliance() == wpilib.DriverStation.Alliance.kRed:
        #     self.tagList = redTags
        # else:
        #     self.tagList = blueTags
        self.tagList = redTags

        print("started aiming")

    def execute(self) -> None:
        #grab raw tag id from camera
        raw_tags = self.camera.getRawFiducials()

        best_tx = None
        max_area = -1.0


        #loops through tag ids currently visible (skips by 7 because data in between is other stuff per id)
        for i in range(0, len(raw_tags), 7):
            tag_id = int(raw_tags[i])
            area = raw_tags[i+3]

            #use biggest of priority ids as main target
            if tag_id in self.tagList:
                if area > max_area:
                    max_area = area
                    best_tx = raw_tags[i+1]

            #if found offset rotate towards offset
            if best_tx != None:
                #need to check wpilib's orientationController idk if this works
                rotation_output = self.driveSubsystem.orientationController.calculate(best_tx, 0) 
                rotation_output = max(min(rotation_output, 0.15), -0.15)
                self.driveSubsystem.driveArcade(0, rotation_output)
            else:
                self.driveSubsystem.driveArcade(0, 0)



    def isFinished(self) -> bool:
        #check if finished
        raw_tags = self.camera.getRawFiducials()
        for i in range(0, len(raw_tags), 7):
            if int(raw_tags[i]) in self.tagList:
                return False
        return True
    
    def end(self, interrupted: bool):
        #stop motors
        self.driveSubsystem.driveArcade(0, 0)
        print("finished aiming")