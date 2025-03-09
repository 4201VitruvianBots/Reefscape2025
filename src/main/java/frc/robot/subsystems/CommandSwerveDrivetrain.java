package frc.robot.subsystems;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.SignalLogger;
import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.swerve.SwerveDrivetrainConstants;
import com.ctre.phoenix6.swerve.SwerveModuleConstants;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.pathplanner.lib.util.DriveFeedforwards;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
// import frc.robot.commands.swerve.PositionPIDCommand;
import frc.robot.constants.SWERVE;
import frc.robot.constants.VISION;
import frc.robot.constants.VISION.TRACKING_STATE;
import frc.robot.generated.V2Constants.TunerSwerveDrivetrain;
import java.io.IOException;
import java.util.function.Supplier;
import org.json.simple.parser.ParseException;
import org.team4201.codex.subsystems.SwerveSubsystem;
import org.team4201.codex.utils.CtreUtils;
import org.team4201.codex.utils.ModuleMap;
import org.team4201.codex.utils.TrajectoryUtils;

/**
 * Class that extends the Phoenix 6 SwerveDrivetrain class and implements Subsystem so it can easily
 * be used in command-based projects.
 */
public class CommandSwerveDrivetrain extends TunerSwerveDrivetrain implements SwerveSubsystem {
  // private final SwerveDrive m_swerveDrive;
  private Vision m_vision;

  private final TalonFX[] driveMotors = {
    getModule(0).getDriveMotor(),
    getModule(1).getDriveMotor(),
    getModule(2).getDriveMotor(),
    getModule(3).getDriveMotor()
  };

  private final TalonFX[] steerMotors = {
    getModule(0).getDriveMotor(),
    getModule(1).getDriveMotor(),
    getModule(2).getDriveMotor(),
    getModule(3).getDriveMotor()
  };

  private static final double kSimLoopPeriod = 0.005; // 5 ms
  private Notifier m_simNotifier = null;
  private double m_lastSimTime;

  /* Blue alliance sees forward as 0 degrees (toward red alliance wall) */
  private static final Rotation2d kBlueAlliancePerspectiveRotation = Rotation2d.kZero;
  /* Red alliance sees forward as 180 degrees (toward blue alliance wall) */
  private static final Rotation2d kRedAlliancePerspectiveRotation = Rotation2d.k180deg;
  /* Keep track if we've ever applied the operator perspective before or not */
  private boolean m_hasAppliedOperatorPerspective = false;

  private TrajectoryUtils m_trajectoryUtils;

  /** Swerve request to apply during robot-centric path following */
  private final SwerveRequest.ApplyRobotSpeeds m_pathApplyRobotSpeeds =
      new SwerveRequest.ApplyRobotSpeeds();

  /* Swerve requests to apply during SysId characterization */
  private final SwerveRequest.SysIdSwerveTranslation m_translationCharacterization =
      new SwerveRequest.SysIdSwerveTranslation();
  private final SwerveRequest.SysIdSwerveSteerGains m_steerCharacterization =
      new SwerveRequest.SysIdSwerveSteerGains();
  private final SwerveRequest.SysIdSwerveRotation m_rotationCharacterization =
      new SwerveRequest.SysIdSwerveRotation();

  /* SysId routine for characterizing translation. This is used to find PID gains for the drive motors. */
  private final SysIdRoutine m_sysIdRoutineTranslation =
      new SysIdRoutine(
          new SysIdRoutine.Config(
              null, // Use default ramp rate (1 V/s)
              Volts.of(4), // Reduce dynamic step voltage to 4 V to prevent brownout
              null, // Use default timeout (10 s)
              // Log state with SignalLogger class
              state -> SignalLogger.writeString("SysIdTranslation_State", state.toString())),
          new SysIdRoutine.Mechanism(
              output -> setControl(m_translationCharacterization.withVolts(output)), null, this));

  /* SysId routine for characterizing steer. This is used to find PID gains for the steer motors. */
  private final SysIdRoutine m_sysIdRoutineSteer =
      new SysIdRoutine(
          new SysIdRoutine.Config(
              null, // Use default ramp rate (1 V/s)
              Volts.of(7), // Use dynamic voltage of 7 V
              null, // Use default timeout (10 s)
              // Log state with SignalLogger class
              state -> SignalLogger.writeString("SysIdSteer_State", state.toString())),
          new SysIdRoutine.Mechanism(
              volts -> setControl(m_steerCharacterization.withVolts(volts)), null, this));

  /*
   * SysId routine for characterizing rotation.
   * This is used to find PID gains for the FieldCentricFacingAngle HeadingController.
   * See the documentation of SwerveRequest.SysIdSwerveRotation for info on importing the log to SysId.
   */
  private final SysIdRoutine m_sysIdRoutineRotation =
      new SysIdRoutine(
          new SysIdRoutine.Config(
              /* This is in radians per second^2, but SysId only supports "volts per second" */
              Volts.of(Math.PI / 6).per(Second),
              /* This is in radians per second, but SysId only supports "volts" */
              Volts.of(Math.PI),
              null, // Use default timeout (10 s)
              // Log state with SignalLogger class
              state -> SignalLogger.writeString("SysIdRotation_State", state.toString())),
          new SysIdRoutine.Mechanism(
              output -> {
                /* output is actually radians per second, but SysId only supports "volts" */
                setControl(m_rotationCharacterization.withRotationalRate(output.in(Volts)));
                /* also log the requested output for SysId */
                SignalLogger.writeDouble("Rotational_Rate", output.in(Volts));
              },
              null,
              this));

  /* The SysId routine to test */
  private SysIdRoutine m_sysIdRoutineToApply = m_sysIdRoutineTranslation;

  private final PIDController m_pidController = new PIDController(11.0, 0.0, 0.0);
  private Rotation2d m_targetAngle = new Rotation2d();
  private Rotation2d m_angleToTarget = new Rotation2d();
  private VISION.TRACKING_STATE m_trackingState = VISION.TRACKING_STATE.NONE;
  private SwerveDriveKinematics m_kinematics;

  /**
   * Constructs a CTRE SwerveDrivetrain using the specified constants.
   *
   * <p>This constructs the underlying hardware devices, so users should not construct the devices
   * themselves. If they need the devices, they can access them through getters in the classes.
   *
   * @param drivetrainConstants Drivetrain-wide constants for the swerve drive
   * @param modules Constants for each specific module
   */
  public CommandSwerveDrivetrain(
      SwerveDrivetrainConstants drivetrainConstants, SwerveModuleConstants<?, ?, ?>... modules) {
    super(drivetrainConstants, modules);
    if (Utils.isSimulation()) {
      startSimThread();
    }
    configureAutoBuilder();

    try {
      m_trajectoryUtils = new TrajectoryUtils(this);
    } catch (Exception ex) {
      DriverStation.reportError("Failed to configure TrajectoryUtils", ex.getStackTrace());
    }
  }

  public void initDriveSysid() {
    for (ModuleMap.MODULE_POSITION i : ModuleMap.MODULE_POSITION.values()) {
      var driveMotor = getModule(i.ordinal()).getDriveMotor();
      var turnMotor = getModule(i.ordinal()).getSteerMotor();
      CtreUtils.configureTalonFx(driveMotor, new TalonFXConfiguration());
      CtreUtils.configureTalonFx(turnMotor, new TalonFXConfiguration());
      driveMotor.setNeutralMode(NeutralModeValue.Brake);
      BaseStatusSignal.setUpdateFrequencyForAll(
          250, driveMotor.getPosition(), driveMotor.getVelocity(), driveMotor.getMotorVoltage());

      driveMotor.optimizeBusUtilization();
    }
  }

  /**
   * Constructs a CTRE SwerveDrivetrain using the specified constants.
   *
   * <p>This constructs the underlying hardware devices, so users should not construct the devices
   * themselves. If they need the devices, they can access them through getters in the classes.
   *
   * @param drivetrainConstants Drivetrain-wide constants for the swerve drive
   * @param odometryUpdateFrequency The frequency to run the odometry loop. If unspecified or set to
   *     0 Hz, this is 250 Hz on CAN FD, and 100 Hz on CAN 2.0.
   * @param modules Constants for each specific module
   */
  public CommandSwerveDrivetrain(
      SwerveDrivetrainConstants drivetrainConstants,
      double odometryUpdateFrequency,
      SwerveModuleConstants<?, ?, ?>... modules) {
    super(drivetrainConstants, odometryUpdateFrequency, modules);
    if (Utils.isSimulation()) {
      startSimThread();
    }
    configureAutoBuilder();
  }

  /**
   * Constructs a CTRE SwerveDrivetrain using the specified constants.
   *
   * <p>This constructs the underlying hardware devices, so users should not construct the devices
   * themselves. If they need the devices, they can access them through getters in the classes.
   *
   * @param drivetrainConstants Drivetrain-wide constants for the swerve drive
   * @param odometryUpdateFrequency The frequency to run the odometry loop. If unspecified or set to
   *     0 Hz, this is 250 Hz on CAN FD, and 100 Hz on CAN 2.0.
   * @param odometryStandardDeviation The standard deviation for odometry calculation in the form
   *     [x, y, theta]^T, with units in meters and radians
   * @param visionStandardDeviation The standard deviation for vision calculation in the form [x, y,
   *     theta]^T, with units in meters and radians
   * @param modules Constants for each specific module
   */
  public CommandSwerveDrivetrain(
      SwerveDrivetrainConstants drivetrainConstants,
      double odometryUpdateFrequency,
      Matrix<N3, N1> odometryStandardDeviation,
      Matrix<N3, N1> visionStandardDeviation,
      SwerveModuleConstants<?, ?, ?>... modules) {
    super(
        drivetrainConstants,
        odometryUpdateFrequency,
        odometryStandardDeviation,
        visionStandardDeviation,
        modules);
    if (Utils.isSimulation()) {
      startSimThread();
    }
    configureAutoBuilder();
  }

  public void setChassisSpeeds(ChassisSpeeds chassisSpeeds) {
    setControl(m_pathApplyRobotSpeeds.withSpeeds(chassisSpeeds));
  }

  public void setChassisSpeedsAuto(
      ChassisSpeeds chassisSpeeds, DriveFeedforwards driveFeedforwards) {
    setControl(
        m_pathApplyRobotSpeeds
            .withSpeeds(chassisSpeeds)
            .withWheelForceFeedforwardsX(driveFeedforwards.robotRelativeForcesXNewtons())
            .withWheelForceFeedforwardsY(driveFeedforwards.robotRelativeForcesYNewtons()));
  }

  public void resetGyro(double angle) {
    getPigeon2().setYaw(angle);
  }

  public void setNeutralMode(SWERVE.MOTOR_TYPE type, NeutralModeValue neutralModeValue) {
    switch (type) {
      case ALL -> {
        for (int i = 0; i < driveMotors.length; i++) {
          driveMotors[i].setNeutralMode(neutralModeValue);
          steerMotors[i].setNeutralMode(neutralModeValue);
        }
      }
      case DRIVE -> {
        for (var motor : driveMotors) {
          motor.setNeutralMode(neutralModeValue);
        }
      }
      case STEER -> {
        for (var motor : steerMotors) {
          motor.setNeutralMode(neutralModeValue);
        }
      }
    }
  }

  private void configureAutoBuilder() {
    try {
      var config = RobotConfig.fromGUISettings();
      AutoBuilder.configure(
          () -> getState().Pose, // Supplier of current robot pose
          this::resetPose, // Consumer for seeding pose against auto
          () -> getState().Speeds, // Supplier of current robot speeds
          // Consumer of ChassisSpeeds and feedforwards to drive the robot
          (speeds, feedforwards) ->
              setControl(
                  m_pathApplyRobotSpeeds
                      .withSpeeds(speeds)
                      .withWheelForceFeedforwardsX(feedforwards.robotRelativeForcesXNewtons())
                      .withWheelForceFeedforwardsY(feedforwards.robotRelativeForcesYNewtons())),
          new PPHolonomicDriveController(
              // PID constants for translation
              new PIDConstants(10, 0, 0),
              // PID constants for rotation
              new PIDConstants(7, 0, 0)),
          config,
          // Assume the path needs to be flipped for Red vs Blue, this is normally the case
          () -> DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Red,
          this // Subsystem for requirements
          );
    } catch (Exception ex) {
      DriverStation.reportError(
          "Failed to load PathPlanner config and configure AutoBuilder", ex.getStackTrace());
    }
  }

  public ChassisSpeeds getChassisSpeed() {
    return m_kinematics.toChassisSpeeds(getState().ModuleStates);
  }

  public ChassisSpeeds getFieldVelocity() {
    ChassisSpeeds robotRelativeSpeeds = m_kinematics.toChassisSpeeds(this.getState().ModuleStates);
    return ChassisSpeeds.fromRobotRelativeSpeeds(
        robotRelativeSpeeds, getState().Pose.getRotation());
  }

  public LinearVelocity getVelocityMagnitude(ChassisSpeeds cs) {
    return MetersPerSecond.of(
        new Translation2d(cs.vxMetersPerSecond, cs.vyMetersPerSecond).getNorm());
  }

  public Rotation2d getPathVelocityHeading(ChassisSpeeds cs, Pose2d target) {
    if (getVelocityMagnitude(cs).in(MetersPerSecond) < 0.25) {
      var diff = target.minus(getState().Pose).getTranslation();
      return (diff.getNorm() < 0.01)
          ? target.getRotation()
          : diff.getAngle(); // .rotateBy(Rotation2d.k180deg);
    }
    return new Rotation2d(cs.vxMetersPerSecond, cs.vyMetersPerSecond);
  }

  public TrajectoryUtils getTrajectoryUtils() {
    return m_trajectoryUtils;
  }

  @Override
  public RobotConfig getAutoRobotConfig() {
    try {
      return RobotConfig.fromGUISettings();
    } catch (IOException e) {
      DriverStation.reportWarning(
          "[SwerveDrive] Could not load RobotConfig for autos!", e.getStackTrace());
      throw new RuntimeException(e);
    } catch (ParseException e) {
      DriverStation.reportWarning(
          "[SwerveDrive] Could not parse RobotConfig for autos!", e.getStackTrace());
      throw new RuntimeException(e);
    }
  }

  @Override
  public PIDConstants getAutoTranslationPIDConstants() {
    // TODO: Move PID constants to variables under SWERVE.java
    return new PIDConstants(10, 0, 0);
  }

  @Override
  public PIDConstants getAutoRotationPIDConstants() {
    // TODO: Move PID constants to variables under SWERVE.java
    return new PIDConstants(7, 0, 0);
  }

  /**
   * Returns a command that applies the specified control request to this swerve drivetrain.
   *
   * @param requestSupplier Function returning the request to apply
   * @return Command to run
   */
  public Command applyRequest(Supplier<SwerveRequest> requestSupplier) {
    return run(() -> this.setControl(requestSupplier.get()));
  }

  public void setAngleToTarget(Rotation2d angle) {
    m_angleToTarget = angle;
  }

  public void setTrackingState(VISION.TRACKING_STATE state) {
    if (m_trackingState != state) {
      m_pidController.reset();
      m_trackingState = state;
    }
  }

  public double calculateRotationToTarget() {
    return m_pidController.calculate(
        getState().Pose.getRotation().getRadians(), m_targetAngle.getRadians());
  }

  private void updateTargetAngle() {
    switch (m_trackingState) {
      case BRANCH:
        m_targetAngle = m_angleToTarget;
        if (m_vision != null) m_vision.setTrackingState(m_trackingState);
        break;
      default:
      case NONE:
        break;
    }
  }

  public boolean isTrackingState() {
    if (m_trackingState == TRACKING_STATE.BRANCH) {
      return true;
    } else {
      return false;
    }
  }

  /**
   * Runs the SysId Quasistatic test in the given direction for the routine specified by {@link
   * #m_sysIdRoutineToApply}.
   *
   * @param direction Direction of the SysId Quasistatic test
   * @return Command to run
   */
  public Command sysIdQuasistatic(SysIdRoutine.Direction direction) {
    return m_sysIdRoutineToApply.quasistatic(direction);
  }

  /**
   * Runs the SysId Dynamic test in the given direction for the routine specified by {@link
   * #m_sysIdRoutineToApply}.
   *
   * @param direction Direction of the SysId Dynamic test
   * @return Command to run
   */
  public Command sysIdDynamic(SysIdRoutine.Direction direction) {
    return m_sysIdRoutineToApply.dynamic(direction);
  }

  @Override
  public void periodic() {
    updateTargetAngle();
    /*
     * Periodically try to apply the operator perspective.
     * If we haven't applied the operator perspective before, then we should apply it regardless of DS state.
     * This allows us to correct the perspective in case the robot code restarts mid-match.
     * Otherwise, only check and apply the operator perspective if the DS is disabled.
     * This ensures driving behavior doesn't change until an explicit disable event occurs during testing.
     */
    if (!m_hasAppliedOperatorPerspective || DriverStation.isDisabled()) {
      DriverStation.getAlliance()
          .ifPresent(
              allianceColor -> {
                setOperatorPerspectiveForward(
                    allianceColor == Alliance.Red
                        ? kRedAlliancePerspectiveRotation
                        : kBlueAlliancePerspectiveRotation);
                m_hasAppliedOperatorPerspective = true;
              });
    }
    // poseEstimator.update(getPigeon2().getRotation2d(), getModulePositions());
  }

  private void startSimThread() {
    m_lastSimTime = Utils.getCurrentTimeSeconds();

    /* Run simulation at a faster rate so PID gains behave more reasonably */
    m_simNotifier =
        new Notifier(
            () -> {
              final double currentTime = Utils.getCurrentTimeSeconds();
              double deltaTime = currentTime - m_lastSimTime;
              m_lastSimTime = currentTime;

              /* use the measured time delta, get battery voltage from WPILib */
              updateSimState(deltaTime, RobotController.getBatteryVoltage());
            });
    m_simNotifier.startPeriodic(kSimLoopPeriod);
  }

  @Override
  public void addVisionMeasurement(Pose2d pose, double timestampSeconds) {
    super.addVisionMeasurement(pose, Utils.fpgaToCurrentTime(timestampSeconds));
  }

  @Override
  public void addVisionMeasurement(
      Pose2d pose, double timestampSeconds, Matrix<N3, N1> standardDevs) {
    super.addVisionMeasurement(pose, Utils.fpgaToCurrentTime(timestampSeconds), standardDevs);
  }
}
