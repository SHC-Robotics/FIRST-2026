import ntcore
import commands2
import wpilib

class VisionCamrea(commands2.Subsystem):
    def __init__(self, cameraName: str) -> None:
        super().__init__()

        self.cameraName = cameraName

        instance = ntcore.NetworkTableInstance.getDefault()
        self.table = instance.getTable(self.cameraName)
        self._path = self.table.getPath()

        self.pipelineIndexRequest = self.table.getDoubleTopic("pipeline").publish()
        self.pipelineIndex = self.table.getDoubleTopic("getpipe").getEntry(-1)
        self.hb = self.table.getIntegerTopic("hb").getEntry(0)

        self.lastHeartbeat = 0
        self.lastHeartbeatTime = 0
        self.heartbeating = False
        self.ticked = False

        self.localizerSubscribed = False

    def setPipeline(self, index: int):
        self.pipelineIndexRequest.set(float(index))

    def getPipeline(self) -> int:
        return int(self.pipelineIndex.get(-1))

    def addLocalizer(self):
        if self.localizerSubscribed:
            return

        self.localizerSubscribed = True

        # Set the network tables needed by MegaTag2
        self.robotOrientationSetRequest = self.table.getDoubleArrayTopic("robot_orientation_set").publish()
        self.cameraPoseSetRequest = self.table.getDoubleArrayTopic("camerapose_robotspace_set").publish()
        self.imuModeRequest = self.table.getDoubleArrayTopic("imumode_set").publish()

        # Retrieve robot pose from MegaTag2
        self.botPose = self.table.getDoubleArrayTopic("botpose_orb_wpiblue").getEntry([])
        self.botPoseFlipped = self.table.getDoubleArrayTopic("botpose_orb_wpired").getEntry([])

    def getHB(self) -> float:
        return self.hb.get()

    def getSecondsSinceLastHeartbeat(self) -> float:
        return wpilib.Timer.getFPGATimestamp() - self.lastHeartbeatTime

    def periodic(self) -> None:
        now = wpilib.Timer.getFPGATimestamp()
        heartbeat = self.getHB()
        self.ticked = False
        if heartbeat != self.lastHeartbeat:
            self.lastHeartbeat = heartbeat
            self.lastHeartbeatTime = now
            self.ticked = True
        heartbeating = now < self.lastHeartbeatTime + 5 # no heartbeat for 5s means stale camera
        if heartbeating != self.heartbeating:
            print(f"Camera {self.cameraName} is " + ("UPDATING" if heartbeating else "NOT UPDATING"))
        self.heartbeating = heartbeating
