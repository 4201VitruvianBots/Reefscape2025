package frc.robot.subsystems;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.net.PortForwarder;
import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructPublisher;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.LimelightHelpers;
import frc.robot.Robot;
import frc.robot.constants.FIELD;
import frc.robot.constants.VISION;
import java.util.Arrays;
// import frc.robot.simulation.FieldSim;
import org.photonvision.PhotonCamera;
import org.photonvision.simulation.VisionSystemSim;
import org.team4201.codex.simulation.FieldSim;

public class Vision extends SubsystemBase {
  private CommandSwerveDrivetrain m_swerveDriveTrain;
  private FieldSim m_fieldSim;

  private VisionSystemSim visionSim;

  private boolean m_localized;
  private boolean doRejectUpdateLLF = false;
  private boolean doRejectUpdateLLB = false;
  private VISION.TRACKING_STATE trackingState = VISION.TRACKING_STATE.NONE;

  // NetworkTables publisher setup
  private final NetworkTableInstance inst = NetworkTableInstance.getDefault();

  /* Robot swerve drive state */
  private final NetworkTable table = inst.getTable("LimelightPoseEstimate");
  private final StructPublisher<Pose2d> estPoseLLF =
      table.getStructTopic("estPoseLLF", Pose2d.struct).publish();
  private final StructPublisher<Pose2d> lastUsedPoseLLF =
      table.getStructTopic("lastUsedPoseLLF", Pose2d.struct).publish();
  private final DoublePublisher estTimeStampLLF = table.getDoubleTopic("estTimeStampLLF").publish();
  private final DoublePublisher estTimeStampLLB = table.getDoubleTopic("estTimeStampLLB").publish();
  private final StructPublisher<Pose2d> estPoseLLB =
      table.getStructTopic("estPoseLLB", Pose2d.struct).publish();
  private final StructPublisher<Pose2d> lastUsedPoseLLB =
      table.getStructTopic("lastUsedPoseLLB", Pose2d.struct).publish();

  public Vision() {
    // Port Forwarding to access limelight on USB Ethernet
    for (int port = 5800; port <= 5809; port++) {
      PortForwarder.add(port, VISION.CAMERA_SERVER.LIMELIGHTF.toString(), port);
    }

    for (int port = 5800; port <= 5809; port++) {
      PortForwarder.add(port + 10, VISION.CAMERA_SERVER.LIMELIGHTB.toString(), port);
    }
  }

  public void registerSwerveDrive(CommandSwerveDrivetrain swerveDriveTrain) {
    m_swerveDriveTrain = swerveDriveTrain;
  }

  public void registerFieldSim(FieldSim fieldSim) {
    m_fieldSim = fieldSim;
  }

  public void setTrackingState(VISION.TRACKING_STATE state) {
    trackingState = state;
  }

  public boolean isCameraConnected(PhotonCamera camera) {
    return camera.isConnected();
  }

  private void updateAngleToBranch() {
    DriverStation.getAlliance()
        .ifPresent(
            a -> {
              Pose2d[] robotToBranch = {m_swerveDriveTrain.getState().Pose, new Pose2d()};
              switch (a) {
                case Red ->
                    robotToBranch[1] = robotToBranch[0].nearest(Arrays.asList(FIELD.RED_BRANCHES));
                case Blue ->
                    robotToBranch[1] = robotToBranch[0].nearest(Arrays.asList(FIELD.BLUE_BRANCHES));
              }
              m_fieldSim.addPoses("LineToNearestBranch", robotToBranch);
              m_swerveDriveTrain.setAngleToTarget(
                  m_swerveDriveTrain
                      .getState()
                      .Pose
                      .getTranslation()
                      .minus(robotToBranch[1].getTranslation())
                      .getAngle()
                  // .minus(Rotation2d.k180deg)
                  );
            });
  }

  public boolean getInitialLocalization() {
    return m_localized;
  }

  public void resetInitialLocalization() {
    m_localized = false;
  }

  public void updateSmartDashboard() {
    SmartDashboard.putBoolean("rejectionUpdateLLF", doRejectUpdateLLF);
    SmartDashboard.putBoolean("rejectionUpdateLLB", doRejectUpdateLLB);
  }

  private void updateLog() {}

  @Override
  public void periodic() {

    LimelightHelpers.PoseEstimate limelightMeasurementCam1;
    // cheking when robot is disabled we us mega tag 1 and when robot is enabled we use mega tag 2
    // When we put the robot on the field we don't know our gyro angle  so we use mega tag 1 to get
    // the position
    if (DriverStation.isDisabled()) {
      // get limelight pose estimate using mega tag 1
      limelightMeasurementCam1 = LimelightHelpers.getBotPoseEstimate_wpiBlue("limelight-f");

      if (limelightMeasurementCam1 != null && limelightMeasurementCam1.tagCount >= 2) {
        m_swerveDriveTrain.resetPose(limelightMeasurementCam1.pose);

        SmartDashboard.putBoolean("updateInitalPose", true);
      } else {
        SmartDashboard.putBoolean("updateInitalPose", false);
      }
    } else {
      limelightMeasurementCam1 =
          LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2("limelight-f");
    }
    m_swerveDriveTrain.setVisionMeasurementStdDevs(VecBuilder.fill(.2, .2, 9999999));

    if (limelightMeasurementCam1 == null) {
      DriverStation.reportWarning("LimelightA is not connected", true);

    } else {
      estPoseLLF.set(limelightMeasurementCam1.pose);
      estTimeStampLLF.set(limelightMeasurementCam1.timestampSeconds);

      if (limelightMeasurementCam1.tagCount == 0) {
        doRejectUpdateLLF = true;
      }
      if (limelightMeasurementCam1.timestampSeconds == 0) {
        doRejectUpdateLLF = true;
      }
      if (limelightMeasurementCam1.pose.getTranslation().equals(Translation2d.kZero)) {
        doRejectUpdateLLF = true;
      }
      if (limelightMeasurementCam1.tagCount == 1) {
        if (limelightMeasurementCam1.rawFiducials[0].ambiguity > 0.7) {
          doRejectUpdateLLF = true;
        }
        if (limelightMeasurementCam1.rawFiducials[0].distToCamera > 3.0) {
          doRejectUpdateLLF = true;
        }
      }
      if (m_swerveDriveTrain.getPigeon2().getRate() > 360.0) {
        doRejectUpdateLLF = true;
      }

      if (!doRejectUpdateLLF) {
        lastUsedPoseLLF.set(limelightMeasurementCam1.pose);
        m_swerveDriveTrain.addVisionMeasurement(
            limelightMeasurementCam1.pose, limelightMeasurementCam1.timestampSeconds);
      }
    }

    LimelightHelpers.PoseEstimate limelightMeasurementCam2;
    // cheking when robot is disabled we us mega tag 1 and when robot is enabled we use mega tag 2
    // When we put the robot on the field we don't know our gyro angle  so we use mega tag 1 to get
    // the position
    if (DriverStation.isDisabled()) {
      // get limelight pose estimate using mega tag 1
      limelightMeasurementCam2 = LimelightHelpers.getBotPoseEstimate_wpiBlue("limelight-b");

      if (limelightMeasurementCam2 != null && limelightMeasurementCam2.tagCount >= 2) {
        m_swerveDriveTrain.resetPose(limelightMeasurementCam2.pose);
      }
    } else {
      limelightMeasurementCam2 =
          LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2("limelight-b");
    }

    m_swerveDriveTrain.setVisionMeasurementStdDevs(VecBuilder.fill(.2, .2, 9999999));

    if (limelightMeasurementCam2 == null) {
      DriverStation.reportWarning("LimelightB is not connected", true);
    } else {
      estPoseLLB.set(limelightMeasurementCam2.pose);
      estTimeStampLLB.set(limelightMeasurementCam2.timestampSeconds);

      if (limelightMeasurementCam2.tagCount == 0) {
        doRejectUpdateLLB = true;
      }
      if (limelightMeasurementCam2.timestampSeconds == 0) {
        doRejectUpdateLLB = true;
      }
      if (limelightMeasurementCam2.pose.getTranslation().equals(Translation2d.kZero)) {
        doRejectUpdateLLB = true;
      }
      if (limelightMeasurementCam2.tagCount == 1) {
        if (limelightMeasurementCam2.rawFiducials[0].ambiguity > 0.7) {
          doRejectUpdateLLB = true;
        }
        if (limelightMeasurementCam2.rawFiducials[0].distToCamera > 3.0) {
          doRejectUpdateLLB = true;
        }
      }
      if (m_swerveDriveTrain.getPigeon2().getRate() > 360.0) {
        doRejectUpdateLLB = true;
      }

      if (!doRejectUpdateLLB) {
        lastUsedPoseLLB.set(limelightMeasurementCam2.pose);
        m_swerveDriveTrain.addVisionMeasurement(
            limelightMeasurementCam2.pose, limelightMeasurementCam2.timestampSeconds);
      }
    }
    LimelightHelpers.SetIMUMode("limelight-f", 0);
    LimelightHelpers.SetRobotOrientation(
        "limelight-f",
        m_swerveDriveTrain.getState().Pose.getRotation().getDegrees(),
        0,
        0,
        0,
        0,
        0);

    LimelightHelpers.SetIMUMode("limelight-b", 0);
    LimelightHelpers.SetRobotOrientation(
        "limelight-b",
        m_swerveDriveTrain.getState().Pose.getRotation().getDegrees(),
        0,
        0,
        0,
        0,
        0);

    updateSmartDashboard();
  }

  @Override
  public void simulationPeriodic() {
    // if (m_swerveDriveTrain != null) {
    // visionSim.update(m_swerveDriveTrain.getState().Pose);
    // visionSim.getDebugField().setRobotPose(m_swerveDriveTrain.getState().Pose);
    // }
  }

  public Field2d getSimDebugField() {
    if (!Robot.isSimulation()) return null;
    return visionSim.getDebugField();
  }
}
