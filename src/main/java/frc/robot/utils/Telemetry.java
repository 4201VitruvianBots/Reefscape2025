package frc.robot.utils;

import com.ctre.phoenix6.SignalLogger;
import com.ctre.phoenix6.swerve.SwerveDrivetrain.SwerveDriveState;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.networktables.*;
import frc.robot.constants.SWERVE;

public class Telemetry {
  private final double m_maxSpeed = SWERVE.kMaxSpeedMetersPerSecond;

  // TODO: Re-implement
  //    private FieldSim m_fieldSim;
  //    private final SwerveModuleVisualizer[] m_moduleVisualizer = {
  //            new SwerveModuleVisualizer(ModuleMap.MODULE_POSITION.FRONT_LEFT.name(), m_maxSpeed),
  //            new SwerveModuleVisualizer(ModuleMap.MODULE_POSITION.FRONT_RIGHT.name(),
  // m_maxSpeed),
  //            new SwerveModuleVisualizer(ModuleMap.MODULE_POSITION.BACK_LEFT.name(), m_maxSpeed),
  //            new SwerveModuleVisualizer(ModuleMap.MODULE_POSITION.BACK_RIGHT.name(), m_maxSpeed)
  //    };

  private final Pose2d[] m_swerveModulePoses = {
    new Pose2d(), new Pose2d(), new Pose2d(), new Pose2d(),
  };
  private final Transform2d[] m_moduleTransforms = new Transform2d[4];
  private final double[] m_moduleAngles = new double[4];

  /* What to publish over networktables for telemetry */
  private final NetworkTableInstance inst = NetworkTableInstance.getDefault();

  /* Robot swerve drive state */
  private final NetworkTable driveStateTable = inst.getTable("DriveState");
  private final StructPublisher<Pose2d> drivePose =
      driveStateTable.getStructTopic("Pose", Pose2d.struct).publish();
  private final StructPublisher<ChassisSpeeds> driveSpeeds =
      driveStateTable.getStructTopic("Speeds", ChassisSpeeds.struct).publish();
  private final StructArrayPublisher<SwerveModuleState> driveModuleStates =
      driveStateTable.getStructArrayTopic("ModuleStates", SwerveModuleState.struct).publish();
  private final StructArrayPublisher<SwerveModuleState> driveModuleTargets =
      driveStateTable.getStructArrayTopic("ModuleTargets", SwerveModuleState.struct).publish();
  private final StructArrayPublisher<SwerveModulePosition> driveModulePositions =
      driveStateTable.getStructArrayTopic("ModulePositions", SwerveModulePosition.struct).publish();
  private final DoublePublisher driveTimestamp =
      driveStateTable.getDoubleTopic("Timestamp").publish();
  private final DoublePublisher driveOdometryFrequency =
      driveStateTable.getDoubleTopic("OdometryFrequency").publish();

  /* Robot pose for field positioning */
  private final NetworkTable table = inst.getTable("Pose");
  private final DoubleArrayPublisher fieldPub = table.getDoubleArrayTopic("robotPose").publish();
  private final StringPublisher fieldTypePub = table.getStringTopic(".type").publish();

  private final double[] m_poseArray = new double[3];
  private final double[] m_moduleStatesArray = new double[8];
  private final double[] m_moduleTargetsArray = new double[8];

  /** Construct a telemetry object */
  public Telemetry() {}

  //    public void registerFieldSim(FieldSim fieldSim) {
  //        m_fieldSim = fieldSim;
  //    }

  /* Accept the swerve drive state and telemeterize it to SmartDashboard */
  public void telemeterize(SwerveDriveState state) {
    /* Telemeterize the swerve drive state */
    drivePose.set(state.Pose);
    driveSpeeds.set(state.Speeds);
    driveModuleStates.set(state.ModuleStates);
    driveModuleTargets.set(state.ModuleTargets);
    driveModulePositions.set(state.ModulePositions);
    driveTimestamp.set(state.Timestamp);
    driveOdometryFrequency.set(1.0 / state.OdometryPeriod);

    /* Also write to log file */
    m_poseArray[0] = state.Pose.getX();
    m_poseArray[1] = state.Pose.getY();
    m_poseArray[2] = state.Pose.getRotation().getDegrees();
    for (int i = 0; i < 4; ++i) {
      m_moduleStatesArray[i * 2 + 0] = state.ModuleStates[i].angle.getRadians();
      m_moduleStatesArray[i * 2 + 1] = state.ModuleStates[i].speedMetersPerSecond;
      m_moduleTargetsArray[i * 2 + 0] = state.ModuleTargets[i].angle.getRadians();
      m_moduleTargetsArray[i * 2 + 1] = state.ModuleTargets[i].speedMetersPerSecond;
    }

    SignalLogger.writeDoubleArray("DriveState/Pose", m_poseArray);
    SignalLogger.writeDoubleArray("DriveState/ModuleStates", m_moduleStatesArray);
    SignalLogger.writeDoubleArray("DriveState/ModuleTargets", m_moduleTargetsArray);
    SignalLogger.writeDouble("DriveState/OdometryPeriod", state.OdometryPeriod, "seconds");

    /* Telemeterize the pose to a Field2d */
    fieldTypePub.set("Field2d");
    fieldPub.set(m_poseArray);

    // TODO: Re-impelement
    //        if (m_fieldSim != null) {
    //            for (ModuleMap.MODULE_POSITION i : ModuleMap.MODULE_POSITION.values()) {
    //                m_moduleVisualizer[i.ordinal()].update(state.ModuleStates[i.ordinal()]);
    //                m_moduleTransforms[i.ordinal()] =
    //                        new Transform2d(
    //                                SWERVE.DRIVE.kModuleTranslations.get(i),
    // state.ModuleStates[i.ordinal()].angle);
    //                m_swerveModulePoses[i.ordinal()] =
    // pose.transformBy(m_moduleTransforms[i.ordinal()]);
    //            }
    //
    //            m_fieldSim.updateRobotPose(pose);
    //            m_fieldSim.updateSwervePoses(m_swerveModulePoses);
    //        }
  }
}
