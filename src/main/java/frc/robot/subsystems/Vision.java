package frc.robot.subsystems;

import edu.wpi.first.epilogue.NotLogged;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.net.PortForwarder;
import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructPublisher;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.LimelightHelpers;
import frc.robot.constants.FIELD;
import frc.robot.constants.ROBOT;
import frc.robot.constants.ROBOT.GAME_PIECE;
import frc.robot.constants.VISION.CAMERA_SERVER;
import org.photonvision.simulation.VisionSystemSim;
import org.team4201.codex.simulation.FieldSim;

public class Vision extends SubsystemBase {
  @NotLogged private CommandSwerveDrivetrain m_swerveDriveTrain;
  @NotLogged private FieldSim m_fieldSim;

  @NotLogged private VisionSystemSim visionSim;

  private boolean m_localized = false;

  // @Logged(name = "Selected Game Piece", importance = Logged.Importance.CRITICAL)
  private ROBOT.GAME_PIECE m_selectedGamePiece = ROBOT.GAME_PIECE.CORAL;

  private Pose2d nearestPose = Pose2d.kZero;
  private final Pose2d[] robotToTarget = {Pose2d.kZero, Pose2d.kZero};

  // NetworkTables publisher setup
  @NotLogged private final NetworkTableInstance inst = NetworkTableInstance.getDefault();

  /* Robot swerve drive state */
  @NotLogged private final NetworkTable table = inst.getTable("LimelightPoseEstimate");

  @NotLogged
  private final StructPublisher<Pose2d> estPoseLLF =
      table.getStructTopic("estPoseLLA", Pose2d.struct).publish();

  @NotLogged
  private final DoublePublisher estTimeStamp = table.getDoubleTopic("estTimeStamp").publish();

  @NotLogged
  private final StructPublisher<Pose2d> estPoseLLB =
      table.getStructTopic("estPoseLLB", Pose2d.struct).publish();

  public Vision() {
    // Port Forwarding to access limelight on USB Ethernet
    for (int port = 5800; port <= 5809; port++) {
      PortForwarder.add(port, CAMERA_SERVER.limelightF.toString(), port);
      PortForwarder.add(port + 10, CAMERA_SERVER.limelightB.toString(), port);
    }
  }

  public void registerSwerveDrive(CommandSwerveDrivetrain swerveDriveTrain) {
    m_swerveDriveTrain = swerveDriveTrain;
  }

  public void registerFieldSim(FieldSim fieldSim) {
    m_fieldSim = fieldSim;
  }

  public GAME_PIECE getSelectedGamePiece() {
    return m_selectedGamePiece;
  }

  public boolean isGamePieceCoral() {
    return m_selectedGamePiece == GAME_PIECE.CORAL;
  }

  public boolean isGamePieceAlgae() {
    return m_selectedGamePiece == GAME_PIECE.ALGAE;
  }

  public void setSelectedGamePiece(GAME_PIECE gamePiece) {
    m_selectedGamePiece = gamePiece;
  }

  private void updateNearestScoringTarget() {
    robotToTarget[0] = m_swerveDriveTrain.getState().Pose;
    if (isGamePieceAlgae()) {
      if (Controls.isBlueAlliance()) {
        nearestPose = robotToTarget[0].nearest(FIELD.BLUE_ALGAE_BRANCHES);
      } else {
        nearestPose = robotToTarget[0].nearest(FIELD.RED_ALGAE_BRANCHES);
      }
      robotToTarget[1] = FIELD.ALGAE_TARGETS.getAlgaePoseToTargetPose(nearestPose);
    } else {
      if (Controls.isBlueAlliance()) {
        nearestPose = robotToTarget[0].nearest(FIELD.BLUE_CORAL_BRANCHES);
      } else {
        nearestPose = robotToTarget[0].nearest(FIELD.RED_CORAL_BRANCHES);
      }
      robotToTarget[1] = FIELD.CORAL_TARGETS.getCoralPoseToTargetPose(nearestPose);
    }
    if (m_fieldSim != null) m_fieldSim.addPoses("LineToNearestTarget", robotToTarget);
  }

  public Pose2d getNearestTargetPose() {
    return robotToTarget[1];
  }

  public boolean getInitialLocalization() {
    return m_localized;
  }

  public void resetInitialLocalization() {
    m_localized = false;
  }

  public boolean processLimelight(String limelightName, StructPublisher<Pose2d> posePublisher) {
    boolean rejectLimelightUpdate = false;

    if (DriverStation.isDisabled()) {
      LimelightHelpers.SetIMUMode(limelightName, 1);
      // TODO: Set limelight camera position on disabled

      // TODO: On disabled, use megaTag1 to reset the robotPose and gyro direction
    }

    LimelightHelpers.SetRobotOrientation(
        limelightName,
        m_swerveDriveTrain.getState().Pose.getRotation().getDegrees(),
        0,
        0,
        0,
        0,
        0);
    LimelightHelpers.PoseEstimate limelightMeasurement =
        LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(limelightName);
    m_swerveDriveTrain.setVisionMeasurementStdDevs(VecBuilder.fill(.7, .7, 9999999));

    if (limelightMeasurement == null) {
      rejectLimelightUpdate = true;
      if (RobotBase.isReal())
        DriverStation.reportWarning(limelightName + " is not connected", true);
    } else {
      posePublisher.set(limelightMeasurement.pose);
      estTimeStamp.set(limelightMeasurement.timestampSeconds);

      if (limelightMeasurement.tagCount == 0) {
        rejectLimelightUpdate = true;
      }
      if (!rejectLimelightUpdate) {
        m_swerveDriveTrain.addVisionMeasurement(
            limelightMeasurement.pose, limelightMeasurement.timestampSeconds);
      }
    }
    SmartDashboard.putBoolean(limelightName + "UpdatedRejected", rejectLimelightUpdate);

    return !rejectLimelightUpdate;
  }

  public void updateSmartDashboard() {}

  @Override
  public void periodic() {
    // limelight f
    boolean llaFSuccess = processLimelight(CAMERA_SERVER.limelightF.toString(), estPoseLLF);

    // limelight b
    boolean llaBSuccess = processLimelight(CAMERA_SERVER.limelightB.toString(), estPoseLLB);

    if (!m_localized) {
      // TODO: Change this to check if the robotPose and both limelight are all close to each other
      m_localized = llaFSuccess && llaBSuccess;
    }

    if (m_swerveDriveTrain != null) {
      updateNearestScoringTarget();
    }

    updateSmartDashboard();

    // if our angular velocity is greater than 360 degrees per second, ignore vision updates
    // if(Math.abs(m_swerveDriveTrain.getPigeon2().getAngularVelocityZDevice().getValueAsDouble()) >
    // 720)
    // {
    //   doRejectUpdateLLA = true;
    // }
    // if (limelightMeasurementCam1.tagCount == 0) {
    //   doRejectUpdateLLA = true;
    // }
  }

  @Override
  public void simulationPeriodic() {
    // if (m_swerveDriveTrain != null) {
    // visionSim.update(m_swerveDriveTrain.getState().Pose);
    // visionSim.getDebugField().setRobotPose(m_swerveDriveTrain.getState().Pose);
    // }
  }
}
