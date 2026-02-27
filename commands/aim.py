import commands2
import wpilib

from constants import AprilTagIds, FuelConstants
from subsystems.candrivesubsystem import CANDriveSubsystem
from subsystems.canfuelsubsystem import CANFuelSubsystem

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
        wpilib.SmartDashboard.putString("Current command", "Aim")

        #get current robotpose from the drive subsystem
        #curentPose = self.driveSubsystem.getPose()

        #should check if primary tag id is in tagList and to turn towards it if it is
        #2 ids so might want to average data between the two to get dead center
        #https://docs.limelightvision.io/docs/docs-limelight/apis/complete-networktables-api

        #can set priorityid to ignore other tags if there's any benefit towards aiming

        # Define target tags depending on alliance
        if wpilib.DriverStation.getAlliance() == wpilib.DriverStation.Alliance.kRed:
            self.tag = AprilTagIds.RED_HUB_CENTER
        else:
            self.tag = AprilTagIds.BLUE_HUB_CENTER

        print(f"started aiming at tag {self.tag}")

    def execute(self) -> None:
        # Grab hub data from camera
        data = self.camera.getHubData(self.tag)

        #grab offset from tag
        tx = data["tx"]

        #else stop aiming
        if not tx:
            self.driveSubsystem.driveArcade(0, 0)
            return

        # need to check wpilib's orientationController idk if this works
        rotation_output = self.driveSubsystem.orientationController.calculate(tx, 0) 
        rotation_output = max(min(rotation_output, 0.15), -0.15)
        self.driveSubsystem.driveArcade(0, rotation_output)

    def isFinished(self) -> bool:
        # Finished if tag goes outside camera FOV or robot is lined up
        data = self.camera.getHubData(self.tag)
        tx = data["tx"]

        if not tx:
            return True

        if tx < ALIGNED_THRESHOLD and tx > -ALIGNED_THRESHOLD:
            return True

        return False
    
    def end(self, interrupted: bool):
        # Stop motors
        self.driveSubsystem.driveArcade(0, 0)
        print(f"finished aiming at {self.tag}")
