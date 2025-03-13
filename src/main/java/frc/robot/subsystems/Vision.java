package frc.robot.subsystems;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.epilogue.NotLogged;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
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
import frc.robot.constants.ROBOT.GAME_PIECE;
import frc.robot.constants.VISION.CAMERA_SERVER;
import frc.robot.utils.LimelightSim;
import org.team4201.codex.simulation.FieldSim;

public class Vision extends SubsystemBase {
  @NotLogged private CommandSwerveDrivetrain m_swerveDriveTrain;
  @NotLogged private FieldSim m_fieldSim;

  // TODO: Re-add this
  @NotLogged private LimelightSim visionSim;

  private boolean m_localized;

  // TODO: Maybe move GAME_PIECE logic to Controls?
  @Logged(name = "Selected Game Piece", importance = Logged.Importance.CRITICAL)
  private GAME_PIECE m_selectedGamePiece = GAME_PIECE.CORAL;

  private boolean m_useLeftTarget;

  private Pose2d nearestObjectPose = Pose2d.kZero;
  private final Pose2d[] robotToTarget = {Pose2d.kZero, Pose2d.kZero};
  private boolean lockTarget = false;

  // NetworkTables publisher setup
  @NotLogged private final NetworkTableInstance inst = NetworkTableInstance.getDefault();
  @NotLogged private final NetworkTable table = inst.getTable("LimelightPoseEstimate");

  @NotLogged
  private final DoublePublisher estTimeStamp = table.getDoubleTopic("estTimeStamp").publish();

  @NotLogged
  private final StructPublisher<Pose2d> estPoseLLF =
      table.getStructTopic("estPoseLLF", Pose2d.struct).publish();

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

  public void setLeftTarget(boolean value) {
    m_useLeftTarget = value;
  }

  private void updateNearestScoringTarget() {
    if (lockTarget) return;
    robotToTarget[0] = m_swerveDriveTrain.getState().Pose;
    if (isGamePieceAlgae()) {
      if (Controls.isBlueAlliance()) {
        nearestObjectPose = robotToTarget[0].nearest(FIELD.BLUE_ALGAE_BRANCHES);
      } else {
        nearestObjectPose = robotToTarget[0].nearest(FIELD.RED_ALGAE_BRANCHES);
      }
      robotToTarget[1] = FIELD.ALGAE_TARGETS.getAlgaePoseToTargetPose(nearestObjectPose);
    } else {
      if (Controls.isBlueAlliance()) {
        if (m_useLeftTarget) {
          nearestObjectPose = robotToTarget[0].nearest(FIELD.BLUE_CORAL_LEFT_BRANCHES);
        } else {
          nearestObjectPose = robotToTarget[0].nearest(FIELD.BLUE_CORAL_RIGHT_BRANCHES);
        }
      } else {
        if (m_useLeftTarget) {
          nearestObjectPose = robotToTarget[0].nearest(FIELD.RED_CORAL_LEFT_BRANCHES);
        } else {
          nearestObjectPose = robotToTarget[0].nearest(FIELD.RED_CORAL_RIGHT_BRANCHES);
        }
      }
      robotToTarget[1] = FIELD.CORAL_TARGETS.getCoralPoseToTargetPose(nearestObjectPose);
    }
    if (m_fieldSim != null) m_fieldSim.addPoses("LineToNearestAlgae", robotToTarget);
  }

  public Pose2d getNearestTargetPose() {
    return robotToTarget[1];
  }

  public boolean getInitialLocalization() {
    return m_localized;
  }

  public void resetInitialLocalization() {
    m_localized = false;

    // Set Swerve Pose to (0, 0) to reset it
    if (m_swerveDriveTrain != null) {
      m_swerveDriveTrain.resetPose(Pose2d.kZero);
    }
  }

  public boolean processLimelight(String limelightName, StructPublisher<Pose2d> posePublisher) {
    if (DriverStation.isDisabled()) {
      // TODO: Determine if we change IMUMode to 0 when not disabled for MegaTag2
      LimelightHelpers.SetIMUMode(limelightName, 1);

      // TODO: Update code values before using this
      //      LimelightHelpers.setCameraPose_RobotSpace(
      //          "limelight-f",
      //          VISION.limelightFPosition.getX(),
      //          VISION.limelightFPosition.getY(),
      //          VISION.limelightFPosition.getZ(),
      //          VISION.limelightFPosition.getRotation().getMeasureX().in(Degrees),
      //          VISION.limelightFPosition.getRotation().getMeasureY().in(Degrees),
      //          VISION.limelightFPosition.getRotation().getMeasureZ().in(Degrees));
      //      LimelightHelpers.setCameraPose_RobotSpace(
      //          "limelight-b",
      //          VISION.limelightBPosition.getX(),
      //          VISION.limelightBPosition.getY(),
      //          VISION.limelightBPosition.getZ(),
      //          VISION.limelightBPosition.getRotation().getMeasureX().in(Degrees),
      //          VISION.limelightBPosition.getRotation().getMeasureY().in(Degrees),
      //          VISION.limelightBPosition.getRotation().getMeasureZ().in(Degrees));
    }

    LimelightHelpers.SetRobotOrientation(
        limelightName,
        m_swerveDriveTrain.getState().Pose.getRotation().getDegrees(),
        0,
        0,
        0,
        0,
        0);
    LimelightHelpers.PoseEstimate limelightMeasurement;
    if (DriverStation.isDisabled()) {
      limelightMeasurement = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(limelightName);
    } else {
      limelightMeasurement = LimelightHelpers.getBotPoseEstimate_wpiBlue(limelightName);
    }

    if (limelightMeasurement == null) {
      if (RobotBase.isReal())
        DriverStation.reportWarning(limelightName + " is not connected", true);
      return true;
    } else {
      // Filter out bad AprilTag vision estimates
      if (limelightMeasurement.timestampSeconds == 0) {
        return true;
      } else if (limelightMeasurement.pose.getTranslation().equals(Translation2d.kZero)) {
        return true;
      } else if (limelightMeasurement.tagCount == 0) {
        return true;
      }

      if (!limelightMeasurement.isMegaTag2) {
        // Filter out bad AprilTag vision estimates
        if (limelightMeasurement.tagCount == 1) {
          if (limelightMeasurement.rawFiducials[0].ambiguity > .7) {
            return true;
          } else if (limelightMeasurement.rawFiducials[0].distToCamera > 3) {
            return true;
          }
          // Set Standard Deviations for MegaTag1
          m_swerveDriveTrain.setVisionMeasurementStdDevs(VecBuilder.fill(.5, .5, 9999999));
        } else {
          // Filter out bad AprilTag vision estimates
          if (m_swerveDriveTrain.getGyroYawRate().abs(DegreesPerSecond) > 720.0) {
            return true;
          } else if (limelightMeasurement.tagCount == 0) {
            return true;
          }
          // Set Standard Deviations for MegaTag2
          m_swerveDriveTrain.setVisionMeasurementStdDevs(VecBuilder.fill(.7, .7, 9999999));
        }
      }
    }
    posePublisher.set(limelightMeasurement.pose);
    estTimeStamp.set(limelightMeasurement.timestampSeconds);
    m_swerveDriveTrain.addVisionMeasurement(
        limelightMeasurement.pose, limelightMeasurement.timestampSeconds);

    // Reset the Swerve Pose with MegaTag1 if we are disabled
    if (DriverStation.isDisabled() && !limelightMeasurement.isMegaTag2) {
      m_swerveDriveTrain.resetPose(limelightMeasurement.pose);
    }

    return false;
  }

  public void setTargetLock(boolean set) {
    lockTarget = set;
  }

  @Logged(name = "On Target", importance = Logged.Importance.CRITICAL)
  public boolean isOnTarget() {
    var translationDelta =
        m_swerveDriveTrain
            .getState()
            .Pose
            .getTranslation()
            .minus(robotToTarget[1].getTranslation())
            .getNorm();
    // TODO: Add Rotation delta
    SmartDashboard.putNumber("Target Translation Delta", translationDelta);

    return translationDelta < Inches.of(2).in(Meters);
  }

  @Override
  public void periodic() {
    // limelight f
    boolean llaFSuccess = !processLimelight(CAMERA_SERVER.limelightF.toString(), estPoseLLF);
    SmartDashboard.putBoolean(CAMERA_SERVER.limelightF + " UpdatedRejected", llaFSuccess);

    // limelight b
    boolean llaBSuccess = !processLimelight(CAMERA_SERVER.limelightB.toString(), estPoseLLB);
    SmartDashboard.putBoolean(CAMERA_SERVER.limelightB + " UpdatedRejected", llaBSuccess);

    if (!m_localized) {
      // TODO: Change this to check if the robotPose and both limelight are all close to each other
      m_localized = llaFSuccess && llaBSuccess;
    }

    if (m_swerveDriveTrain != null) {
      updateNearestScoringTarget();
    }
  }

  @Override
  public void simulationPeriodic() {
    // if (m_swerveDriveTrain != null) {
    // visionSim.update(m_swerveDriveTrain.getState().Pose);
    // visionSim.getDebugField().setRobotPose(m_swerveDriveTrain.getState().Pose);
    // }
  }
}
