import commands2
import wpilib
import ntcore


class VisionCamera(commands2.Subsystem):
    def __init__(self, cameraName: str) -> None:
        super().__init__()

        self.cameraName = _fix_name(cameraName)

        instance = ntcore.NetworkTableInstance.getDefault()
        self.table = instance.getTable(self.cameraName)
        self._path = self.table.getPath()

        # Pipelines are:
        # 1 - AprilTags
        self.pipelineIndexRequest = self.table.getDoubleTopic("pipeline").publish()
        self.pipelineIndex = self.table.getDoubleTopic("getpipe").getEntry(-1)
        # "cl" and "tl" are additional latencies in milliseconds

        self.ledMode = self.table.getIntegerTopic("ledMode").getEntry(-1)
        self.camMode = self.table.getIntegerTopic("camMode").getEntry(-1)
        self.tx = self.table.getDoubleTopic("tx").getEntry(0.0)
        self.ty = self.table.getDoubleTopic("ty").getEntry(0.0)
        self.ta = self.table.getDoubleTopic("ta").getEntry(0.0)
        self.hb = self.table.getIntegerTopic("hb").getEntry(0)
        self.tid = self.table.getIntegerTopic("tid").getEntry(0)
        self.targetpose_cameraspace = self.table.getDoubleArrayTopic(
            "targetpose_cameraspace"
        ).getEntry([])
        self.camerapose_targetspace = self.table.getDoubleArrayTopic(
            "camerapose_targetspace"
        ).getEntry([])

        self.lastHeartbeat = 0
        self.lastHeartbeatTime = 0
        self.heartbeating = False
        self.ticked = False

        self.localizerSubscribed = False

    def addLocalizer(self):
        if self.localizerSubscribed:
            return

        self.localizerSubscribed = True
        # if we want MegaTag2 localizer to work, we need to be publishing two things (to the camera):
        #   1. what robot's yaw is ("yaw=0 degrees" means "facing North", "yaw=90 degrees" means "facing West", etc.)
        #   2. where is this camera sitting on the robot (e.g. y=-0.2 meters to the right, x=0.1 meters fwd from center)
        self.robotOrientationSetRequest = self.table.getDoubleArrayTopic(
            "robot_orientation_set"
        ).publish()
        self.cameraPoseSetRequest = self.table.getDoubleArrayTopic(
            "camerapose_robotspace_set"
        ).publish()
        self.imuModeRequest = self.table.getIntegerTopic(
            "imumode_set"
        ).publish()  # this is only for Limelight 4

        # and we can then receive the localizer results from the camera back
        self.botPose = self.table.getDoubleArrayTopic("botpose_orb_wpiblue").getEntry(
            []
        )
        self.botPoseFlipped = self.table.getDoubleArrayTopic(
            "botpose_orb_wpired"
        ).getEntry([])

    def setPipeline(self, index: int):
        self.pipelineIndexRequest.set(float(index))

    def getPipeline(self) -> int:
        return int(self.pipelineIndex.get(-1))

    def getA(self) -> float:
        return self.ta.get()

    def getX(self) -> float:
        return self.tx.get()

    def getY(self) -> float:
        return self.ty.get()

    def getHB(self) -> float:
        return self.hb.get()

    def getTid(self) -> int:
        return self.tid.get()

    def get_camerapose_targetspace(self):
        return self.camerapose_targetspace.get()

    def get_targetpose_cameraspace(self):
        return self.targetpose_cameraspace.get()

    def hasDetection(self):
        if self.getX() != 0.0 and self.heartbeating:
            return True

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
        heartbeating = (
            now < self.lastHeartbeatTime + 5
        )  # no heartbeat for 5s => stale camera
        if heartbeating != self.heartbeating:
            print(
                f"Camera {self.cameraName} is "
                + ("UPDATING" if heartbeating else "NO LONGER UPDATING")
            )
        self.heartbeating = heartbeating


def _fix_name(name: str):
    if not name:
        name = "limelight"
    return name
