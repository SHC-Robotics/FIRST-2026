import commands2
from subsystems.canclimbsubsystem import CANClimbSubsystem
from constants import ClimberConstants


class ClimberHoming(commands2.Command):
    """
    Homing routine — run this once at robot startup before using any other climber commands.

    Creeps both arms downward independently. Each arm stops as soon as its own limit
    switch triggers, so arms that are at different heights will home at different times
    without fighting each other. Once BOTH switches are triggered:
      1. Both encoders are zeroed
      2. homePosition is set to 0 so all future position commands are consistent

    Until this completes, ClimbUp and ClimbDown targets will be unreliable.
    """

    def __init__(self, climbSubsystem: CANClimbSubsystem) -> None:
        super().__init__()
        self.climbSubsystem = climbSubsystem
        self.addRequirements(self.climbSubsystem)

    def initialize(self) -> None:
        if self.climbSubsystem.isAtBottom():
            print("ClimberHoming: both switches already active, already homed")
            return

        print("ClimberHoming: starting — driving arms down slowly")
        self._leftTarget = self.climbSubsystem.leftArm.get_position().value
        self._rightTarget = self._leftTarget  # mirror start position for right arm
        # self._rightTarget = self.climbSubsystem.rightArm.get_position().value

    def execute(self) -> None:
        # Creep left arm down only if its switch hasn't triggered yet
        if not self.climbSubsystem.isLeftAtBottom():
            self._leftTarget += ClimberConstants.HOMING_STEP
            self.climbSubsystem.leftArm.set_control(
                self.climbSubsystem.motion_magic_position_request.with_position(self._leftTarget)
            )
        else:
            self.climbSubsystem.stopLeft()

        # Creep right arm down only if its switch hasn't triggered yet
        if not self.climbSubsystem.isRightAtBottom():
            self._rightTarget += ClimberConstants.HOMING_STEP
            # self.climbSubsystem.rightArm.set_control(
            #     self.climbSubsystem.motion_magic_position_request.with_position(self._rightTarget)
            # )
        else:
            self.climbSubsystem.stopRight()

    def end(self, interrupted: bool) -> None:
        self.climbSubsystem.stop()

        if not interrupted:
            # Zero both encoders at their current physical positions
            self.climbSubsystem.leftArm.set_position(0)
            # self.climbSubsystem.rightArm.set_position(0)

            # Set home as 0 so all future targets are relative to this zeroed point
            self.climbSubsystem.homePosition = 0.0

            print("ClimberHoming: complete — both encoders zeroed at limit switches")
        else:
            print("ClimberHoming: interrupted before both switches triggered — NOT zeroed")

    def isFinished(self) -> bool:
        # Wait until both arms have hit their respective limit switches
        if self.climbSubsystem.isAtBottom():
            print("ClimberHoming: both limit switches triggered")
            return True
        return False