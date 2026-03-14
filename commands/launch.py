import commands2
import wpilib

from constants import FuelConstants, AprilTagIds
from subsystems.canfuelsubsystem import CANFuelSubsystem


class Launch(commands2.Command):
    """
    A Command that represents the complete action of launching fuel.
    Requires the fuel subsystem.
    """

    def __init__(self, fuelSubsystem: CANFuelSubsystem, visionCamera) -> None:
        super().__init__()

        self.fuelSubsystem = fuelSubsystem
        self.addRequirements(self.fuelSubsystem)

        self.camera = visionCamera


        #TODO: PLEASE FIND ACTUAL VALUES FOR THIS; distance (meters) : percentage of max voltage
        self.speedTable = {
            0.8: 0.65,
            1.2: 0.69,
            1.6: 0.73,
            2: 0.77,
            2.4: 0.81,
            2.8: 0.85
        }


        # if wpilib.DriverStation.getAlliance() == wpilib.DriverStation.Alliance.kRed:
        #     self.tag = AprilTagIds.RED_HUB_CENTER
        # else:
        #     self.tag = AprilTagIds.BLUE_HUB_CENTER

    def initialize(self) -> None:
        wpilib.SmartDashboard.putString("Current command", "Launch")

        data = self.camera.getHubData(self.tag)
        distance = data["dist"]

        # multiplier = self.findSpeed(distance)
        multiplier = self.fuelSubsystem.multiplier

        print(f"Distance: {distance}")
        print(f"Multiplier: {distance}")

        self.fuelSubsystem.setIntakeLauncherRoller(
            FuelConstants.LAUNCHING_LAUNCHER_VOLTAGE * multiplier
        )
        self.fuelSubsystem.setFeederRoller(
            FuelConstants.LAUNCHING_FEEDER_VOLTAGE
        )

    
    def isFinished(self) -> bool:
        return False

    def end(self, interrupted: bool) -> None:
        self.fuelSubsystem.stop()
    
    def findSpeed(self, currentDistance):
        speedTable = self.speedTable

        #list of speeds for index values of least to greatest
        distances = sorted(self.speedTable.keys())

        #no dist means choose greatest speed; might be redundant
        if currentDistance is None:
            return speedTable[distances[-1]]
        
        #closer than closest choose lower speed
        if currentDistance < distances[0]:
            return speedTable[distances[0]]
        
        for dist in distances:
            if dist > currentDistance:
                return speedTable[dist]
            

        #defaults to greatest distance if none in table higher than current
        return speedTable[distances[-1]]
    

class StopLaunch(commands2.Command):
    def __init__(self, fuelSubsystem: CANFuelSubsystem) -> None:
        super().__init__()

        self.fuelSubsystem = fuelSubsystem
        self.addRequirements(self.fuelSubsystem)

    def initialize(self) -> None:
        self.fuelSubsystem.stop()

class ChangeLaunchSpeed(commands2.Command):
    def __init__(self, fuelSubsystem: CANFuelSubsystem, operatorController) -> None:
        super().__init__()

        self.fuelSubsystem = fuelSubsystem
        self.controller = operatorController
        self.addRequirements(self.fuelSubsystem)

    def execute(self) -> None:
        leftTrigger = self.controller.getLeftTriggerAxis()
        rightTrigger = self.controller.getRightTriggerAxis()

        if leftTrigger < 0.1:
            leftTrigger = 0

        if rightTrigger < 0.1:
            rightTrigger = 0

        self.fuelSubsystem.multiplier -= leftTrigger * 0.1
        self.fuelSubsystem.multiplier += rightTrigger * 0.1

        self.fuelSubsystem.multiplier = max(min(self.fuelSubsystem.multiplier, 0.85), 0.65)

        wpilib.SmartDashboard.putNumber("Shooting multiplier", self.fuelSubsystem.multiplier)

    def isFinished(self) -> bool:
        return False
