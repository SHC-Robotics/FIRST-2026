import commands2
from subsystems.canclimbsubsystem import CANClimbSubsystem
from constants import ClimberConstants


class ClimberHoming(commands2.Command):
    """
    Homing routine — run this once at robot startup before using any other climber commands.

    Slowly drives the arms downward until the limit switch trips, then:
      1. Stops the motors
      2. Resets the TalonFX encoder to zero at that position
      3. Sets subsystem.homePosition so all other commands have a consistent reference

    Bind this to robot init or a dedicated "home" button. Until this completes,
    ClimbUp and ClimbDown targets will be unreliable.
    """

    def __init__(self, climbSubsystem: CANClimbSubsystem) -> None:
        super().__init__()
        self.climbSubsystem = climbSubsystem
        self.addRequirements(self.climbSubsystem)

    def initialize(self) -> None:
        # If already homed, nothing to do
        if self.climbSubsystem.isAtBottom():
            print("ClimberHoming: limit switch already active, already homed")
            return

        print("ClimberHoming: starting homing routine — driving arms down slowly")
        # Use a slow rolling target to creep downward without overshooting
        self._rollingTarget = self.climbSubsystem.leftArm.get_position().value

    def execute(self) -> None:
        if self.climbSubsystem.isAtBottom():
            return

        # Creep downward each loop at homing speed (slower than normal manual speed)
        self._rollingTarget += ClimberConstants.HOMING_STEP
        self.climbSubsystem.setMotionMagicPosition(self._rollingTarget)

    def end(self, interrupted: bool) -> None:
        self.climbSubsystem.stop()

        if not interrupted:
            # Zero the encoder at the current physical position
            self.climbSubsystem.leftArm.set_position(0)
            # self.climbSubsystem.rightArm.set_position(0)

            # Record home as 0 so all future targets are relative to this point
            self.climbSubsystem.homePosition = 0.0

            print("ClimberHoming: homing complete — encoder zeroed at limit switch")
        else:
            print("ClimberHoming: interrupted before limit switch was reached — NOT zeroed")

    def isFinished(self) -> bool:
        if self.climbSubsystem.isAtBottom():
            print("ClimberHoming: limit switch triggered")
            return True
        return False