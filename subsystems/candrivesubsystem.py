import commands2

from phoenix6 import hardware, configs, controls, signals
from phoenix6.sim import ChassisReference
from phoenix6.unmanaged import feed_enable
from constants import DriveConstants
from math import pi
from wpimath.estimator import DifferentialDrivePoseEstimator
from wpimath.kinematics import DifferentialDriveKinematics, DifferentialDriveWheelSpeeds, ChassisSpeeds
from wpimath.geometry import Pose2d, Rotation2d
from wpimath.controller import PIDController
from wpimath.system.plant import DCMotor, LinearSystemId
from pathplannerlib.auto import AutoBuilder
from pathplannerlib.controller import PPLTVController
from pathplannerlib.config import RobotConfig
import wpilib
import wpilib.simulation

import navx


"""
Math is from

https://v6.docs.ctr-electronics.com/en/stable/docs/api-reference/device-specific/talonfx/closed-loop-requests.html#converting-from-meters
"""
WHEEL_RADIUS_METERS = 0.0762        # 3 inch wheels
GEARBOX_RATIO       = 8.45
TRACK_WIDTH_METERS  = 0.549275
METERS_PER_ROTATION = (2 * pi * WHEEL_RADIUS_METERS) / GEARBOX_RATIO


class CANDriveSubsystem(commands2.Subsystem):
    def __init__(self) -> None:
        super().__init__()

        # ------------------------------------------------------------------ #
        # Motors                                                               #
        # ------------------------------------------------------------------ #
        self.leftLeader   = hardware.TalonFX(DriveConstants.LEFT_LEADER_ID)
        self.leftFollower  = hardware.TalonFX(DriveConstants.LEFT_FOLLOWER_ID)
        self.rightLeader  = hardware.TalonFX(DriveConstants.RIGHT_LEADER_ID)
        self.rightFollower = hardware.TalonFX(DriveConstants.RIGHT_FOLLOWER_ID)

        # ------------------------------------------------------------------ #
        # Motor configuration                                                  #
        # ------------------------------------------------------------------ #
        config = configs.TalonFXConfiguration()

        slot0       = config.slot0
        slot0.k_p   = 0.01
        slot0.k_v   = 0.136   # 12v / 88 RPS (5.0 m/s max)
        slot0.k_s   = 0.25
        slot0.k_i   = 0
        slot0.k_d   = 0

        self.MAX_RPS = 88   # ~5.0 m/s

        # Apply PID gains to both sides first
        self.leftLeader.configurator.apply(config)
        self.rightLeader.configurator.apply(config)

        self.velocity_request = controls.VelocityVoltage(0).with_slot(0)

        # Apply left inversion
        config.motor_output.inverted = configs.config_groups.InvertedValue.CLOCKWISE_POSITIVE
        self.leftLeader.configurator.apply(config)

        # Apply right inversion
        config.motor_output.inverted = configs.config_groups.InvertedValue.COUNTER_CLOCKWISE_POSITIVE
        self.rightLeader.configurator.apply(config)

        # Followers
        self.leftFollower.set_control(
            controls.Follower(DriveConstants.LEFT_LEADER_ID, signals.MotorAlignmentValue.ALIGNED)
        )
        self.rightFollower.set_control(
            controls.Follower(DriveConstants.RIGHT_LEADER_ID, signals.MotorAlignmentValue.ALIGNED)
        )

        # ------------------------------------------------------------------ #
        # Gyro                                                                 #
        # ------------------------------------------------------------------ #
        self.gyro = navx.AHRS.create_spi()

        # ------------------------------------------------------------------ #
        # Kinematics & Pose Estimator                                          #
        # ------------------------------------------------------------------ #
        left_position_meters  = self.leftLeader.get_position().value  * METERS_PER_ROTATION
        right_position_meters = self.rightLeader.get_position().value * METERS_PER_ROTATION

        self.kinematics = DifferentialDriveKinematics(trackWidth=TRACK_WIDTH_METERS)

        self.poseEstimator = DifferentialDrivePoseEstimator(
            self.kinematics,
            self._getGyroRotation(),
            left_position_meters,
            right_position_meters,
            Pose2d(),
        )

        self.field = wpilib.Field2d()
        wpilib.SmartDashboard.putData("Field", self.field)

        # ------------------------------------------------------------------ #
        # PathPlanner                                                          #
        # ------------------------------------------------------------------ #
        ppConfig = RobotConfig.fromGUISettings()

        AutoBuilder.configure(
            self.getPose,
            self.resetPose,
            self.getRobotRelativeSpeeds,
            lambda speeds, feedforwards: self.driveRobotRelative(speeds),
            PPLTVController(0.02),
            ppConfig,
            shouldFlipPath,
            self
        )

        print(f"AutoBuilder configured: {AutoBuilder.isConfigured()}")

        # ------------------------------------------------------------------ #
        # Orientation PID                                                      #
        # ------------------------------------------------------------------ #
        self.orientationController = PIDController(0.005, 0, 0)
        self.orientationController.enableContinuousInput(-180, 180)
        self.orientationController.setTolerance(1.0)

        # ------------------------------------------------------------------ #
        # Simulation setup                                                     #
        # ------------------------------------------------------------------ #
        if wpilib.RobotBase.isSimulation():
            self._setupSimulation()

    # ---------------------------------------------------------------------- #
    # Simulation                                                               #
    # ---------------------------------------------------------------------- #

    def _setupSimulation(self) -> None:
        """
        Sets up the physics simulation for the drivetrain.

        LinearSystemId.identifyDrivetrainSystem parameters are characterization
        constants (kV/kA) — these are approximate and can be tuned later via
        SysId. They affect simulation fidelity but not real robot behavior.
        """
        self._drivesim = wpilib.simulation.DifferentialDrivetrainSim(
            LinearSystemId.identifyDrivetrainSystem(
                1.98,   # kV linear  (V·s/m)   — tune with SysId
                0.2,    # kA linear  (V·s²/m)
                1.5,    # kV angular (V·s/rad)
                0.3,    # kA angular (V·s²/rad)
            ),
            TRACK_WIDTH_METERS,
            DCMotor.krakenX60(2),   # 2 Krakens per side
            GEARBOX_RATIO,
            WHEEL_RADIUS_METERS,
        )

        # Match physical motor orientations so sim encoder signs agree
        self.leftLeader.sim_state.orientation  = ChassisReference.CLOCKWISE_POSITIVE
        self.rightLeader.sim_state.orientation = ChassisReference.COUNTER_CLOCKWISE_POSITIVE

        # NavX simulation device — index 4 matches create_spi()
        self._sim_gyro  = wpilib.simulation.SimDeviceSim("navX-Sensor[4]")
        self._navx_yaw  = self._sim_gyro.getDouble("Yaw")

    def simulationPeriodic(self) -> None:
        feed_enable(0.04)

        wpilib.simulation.RoboRioSim.setVInVoltage(
            wpilib.simulation.BatterySim.calculate([
                self._drivesim.getCurrentDraw()
            ])
        )

        self.leftLeader.sim_state.set_supply_voltage(wpilib.RobotController.getBatteryVoltage())
        self.rightLeader.sim_state.set_supply_voltage(wpilib.RobotController.getBatteryVoltage())
        self.leftFollower.sim_state.set_supply_voltage(wpilib.RobotController.getBatteryVoltage())
        self.rightFollower.sim_state.set_supply_voltage(wpilib.RobotController.getBatteryVoltage())

        self._drivesim.setInputs(
            self.leftLeader.sim_state.motor_voltage,
            self.rightLeader.sim_state.motor_voltage,
        )

        self._drivesim.update(0.02)

        # Convert feet to meters, then to rotor rotations
        left_pos_m   = self._drivesim.getLeftPositionFeet()   * 0.3048
        right_pos_m  = self._drivesim.getRightPositionFeet()  * 0.3048
        left_vel_ms  = self._drivesim.getLeftVelocityFps()    * 0.3048
        right_vel_ms = self._drivesim.getRightVelocityFps()   * 0.3048

        self.leftLeader.sim_state.set_raw_rotor_position(left_pos_m   / METERS_PER_ROTATION)
        self.leftLeader.sim_state.set_rotor_velocity(    left_vel_ms  / METERS_PER_ROTATION)
        self.rightLeader.sim_state.set_raw_rotor_position(right_pos_m  / METERS_PER_ROTATION)
        self.rightLeader.sim_state.set_rotor_velocity(    right_vel_ms / METERS_PER_ROTATION)

        self._navx_yaw.set(-self._drivesim.getHeading().degrees())

    # ---------------------------------------------------------------------- #
    # Gyro helper                                                              #
    # ---------------------------------------------------------------------- #

    def _getGyroRotation(self) -> Rotation2d:
        """
        Returns gyro heading as a CCW-positive Rotation2d for WPILib.
        NavX getAngle() is CW-positive so we negate it.
        """
        return Rotation2d.fromDegrees(-self.gyro.getAngle())

    # ---------------------------------------------------------------------- #
    # Periodic                                                                 #
    # ---------------------------------------------------------------------- #

    def periodic(self) -> None:
        leftPosition  = self.leftLeader.get_position().value  * METERS_PER_ROTATION
        rightPosition = self.rightLeader.get_position().value * METERS_PER_ROTATION

        pose = self.poseEstimator.update(
            self._getGyroRotation(),
            leftPosition,
            rightPosition,
        )

        self.field.setRobotPose(pose)

        wpilib.SmartDashboard.putNumber("Pose X",           pose.x)
        wpilib.SmartDashboard.putNumber("Pose Y",           pose.y)
        wpilib.SmartDashboard.putNumber("Pose Heading",     pose.rotation().degrees())
        wpilib.SmartDashboard.putNumber("Gyro Heading",     -self.gyro.getAngle())
        wpilib.SmartDashboard.putNumber("Left Position m",  leftPosition)
        wpilib.SmartDashboard.putNumber("Right Position m", rightPosition)
        wpilib.SmartDashboard.putNumber("Left Vel m/s",     self.leftLeader.get_velocity().value  * METERS_PER_ROTATION)
        wpilib.SmartDashboard.putNumber("Right Vel m/s",    self.rightLeader.get_velocity().value * METERS_PER_ROTATION)

    # ---------------------------------------------------------------------- #
    # PathPlanner interface                                                    #
    # ---------------------------------------------------------------------- #

    def getPose(self) -> Pose2d:
        return self.poseEstimator.getEstimatedPosition()

    def resetPose(self, pose: Pose2d) -> None:
        print(f"resetPose: ({pose.x:.2f}, {pose.y:.2f}, {pose.rotation().degrees():.1f}°)  gyro: {-self.gyro.getAngle():.1f}°")
        wpilib.SmartDashboard.putNumber("Reset Pose X",       pose.x)
        wpilib.SmartDashboard.putNumber("Reset Pose Y",       pose.y)
        wpilib.SmartDashboard.putNumber("Reset Pose Heading", pose.rotation().degrees())
        wpilib.SmartDashboard.putNumber("Gyro At Reset",      -self.gyro.getAngle())

        self.poseEstimator.resetPosition(
            self._getGyroRotation(),
            self.leftLeader.get_position().value  * METERS_PER_ROTATION,
            self.rightLeader.get_position().value * METERS_PER_ROTATION,
            pose,
        )

    def getRobotRelativeSpeeds(self) -> ChassisSpeeds:
        speeds = self.kinematics.toChassisSpeeds(self.getWheelSpeeds())
        wpilib.SmartDashboard.putNumber("Reported vx",    speeds.vx)
        wpilib.SmartDashboard.putNumber("Reported omega", speeds.omega)
        return speeds

    def driveRobotRelative(self, speeds: ChassisSpeeds) -> None:
        wheelSpeeds = self.kinematics.toWheelSpeeds(speeds)

        wpilib.SmartDashboard.putNumber("PP Commanded vx",    speeds.vx)
        wpilib.SmartDashboard.putNumber("PP Commanded omega", speeds.omega)
        wpilib.SmartDashboard.putNumber("PP Left m/s",        wheelSpeeds.left)
        wpilib.SmartDashboard.putNumber("PP Right m/s",       wheelSpeeds.right)
        wpilib.SmartDashboard.putNumber("PP Left RPS",        wheelSpeeds.left  / METERS_PER_ROTATION)
        wpilib.SmartDashboard.putNumber("PP Right RPS",       wheelSpeeds.right / METERS_PER_ROTATION)

        self.leftLeader.set_control(
            self.velocity_request.with_velocity(wheelSpeeds.left  / METERS_PER_ROTATION)
        )
        self.rightLeader.set_control(
            self.velocity_request.with_velocity(wheelSpeeds.right / METERS_PER_ROTATION)
        )

    def getHeading(self) -> Rotation2d:
        return self._getGyroRotation()

    def getRotationSpeed(self) -> float:
        """Returns yaw rate in degrees per second, CCW positive."""
        return -self.gyro.getRate()

    def getWheelSpeeds(self) -> DifferentialDriveWheelSpeeds:
        left_speed  = self.leftLeader.get_velocity().value  * METERS_PER_ROTATION
        right_speed = self.rightLeader.get_velocity().value * METERS_PER_ROTATION
        return DifferentialDriveWheelSpeeds(left_speed, right_speed)

    # ---------------------------------------------------------------------- #
    # Teleop drive                                                             #
    # ---------------------------------------------------------------------- #

    def driveVolts(self, leftVolts: float, rightVolts: float) -> None:
        self.leftLeader.set_control(controls.VoltageOut(leftVolts))
        self.rightLeader.set_control(controls.VoltageOut(rightVolts))

    def driveArcade(self, xSpeed: float, zRotation: float) -> None:
        xSpeed = -xSpeed

        left_percent  = xSpeed + zRotation
        right_percent = xSpeed - zRotation

        self.leftLeader.set_control(
            self.velocity_request.with_velocity(left_percent  * self.MAX_RPS)
        )
        self.rightLeader.set_control(
            self.velocity_request.with_velocity(right_percent * self.MAX_RPS)
        )

    def driveToOrientation(self, targetDegrees: float, xSpeed: float = 0) -> None:
        currentDegrees  = self.getPose().rotation().degrees()
        rotationOutput  = self.orientationController.calculate(currentDegrees, targetDegrees)
        self.driveArcade(xSpeed, rotationOutput)

    def isAtTargetOrientation(self) -> bool:
        return self.orientationController.atSetpoint()


def shouldFlipPath() -> bool:
    return wpilib.DriverStation.getAlliance() == wpilib.DriverStation.Alliance.kRed