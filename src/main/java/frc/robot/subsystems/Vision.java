package frc.robot.subsystems;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
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
import frc.robot.constants.VISION.CAMERA_SERVER;
import java.util.Arrays;
import org.photonvision.PhotonCamera;
import org.photonvision.simulation.VisionSystemSim;
import org.team4201.codex.simulation.FieldSim;

public class Vision extends SubsystemBase {
  private CommandSwerveDrivetrain m_swerveDriveTrain;
  private FieldSim m_fieldSim;

  private boolean m_localized;
  private boolean doRejectUpdateLLA = false;
  private boolean doRejectUpdateLLB = false;
  private VISION.TRACKING_STATE trackingState = VISION.TRACKING_STATE.NONE;

  // NetworkTables publisher setup
  private final NetworkTableInstance inst = NetworkTableInstance.getDefault();

  /* Robot swerve drive state */
  private final NetworkTable table = inst.getTable("LimelightPoseEstimate");
  private final StructPublisher<Pose2d> estPoseLLA =
      table.getStructTopic("estPoseLLA", Pose2d.struct).publish();
  private final DoublePublisher estTimeStamp = table.getDoubleTopic("estTimeStamp").publish();
  private final StructPublisher<Pose2d> estPoseLLB =
      table.getStructTopic("estPoseLLB", Pose2d.struct).publish();

  // TODO: Re-add visionSim
  private VisionSystemSim visionSim;

  public Vision() {
    // Port Forwarding to access limelight on USB Ethernet
    for (int port = 5800; port <= 5809; port++) {
      PortForwarder.add(port, CAMERA_SERVER.limelightA.getIp(), port);
      PortForwarder.add(port + 10, CAMERA_SERVER.limelightB.getIp(), port);
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

  public Field2d getSimDebugField() {
    if (!Robot.isSimulation()) return null;
    return visionSim.getDebugField();
  }

  private void updateAngleToBranch() {
    DriverStation.getAlliance()
        .ifPresent(
            a -> {
              Pose2d[] robotToBranch = {m_swerveDriveTrain.getState().Pose, new Pose2d()};
              switch (a) {
                case Red ->
                    robotToBranch[1] =
                        robotToBranch[0].nearest(Arrays.asList(FIELD.RED_CORAL_BRANCHES));
                case Blue ->
                    robotToBranch[1] =
                        robotToBranch[0].nearest(Arrays.asList(FIELD.BLUE_CORAL_BRANCHES));
              }
              m_fieldSim.addPoses("LineToNearestBranch", robotToBranch);
              m_swerveDriveTrain.setTargetAngle(
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
    SmartDashboard.putBoolean("rejectionUpdateLLA", doRejectUpdateLLA);
    SmartDashboard.putBoolean("rejectionUpdateLLB", doRejectUpdateLLB);
  }

  @Override
  public void periodic() {
    // limelight a
    LimelightHelpers.SetRobotOrientation(
        "limelight-a",
        m_swerveDriveTrain.getState().Pose.getRotation().getDegrees(),
        0,
        0,
        0,
        0,
        0);
    LimelightHelpers.PoseEstimate limelightMeasurementCam1 =
        LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2("limelight-a");
    m_swerveDriveTrain.setVisionMeasurementStdDevs(VecBuilder.fill(.5, .5, 9999999));

    if (limelightMeasurementCam1.timestampSeconds == 0) {
      DriverStation.reportWarning("LimelightA not running pose estimation", true);
    } else {
      DriverStation.reportWarning("LimelightA got vison pose", false);
      estPoseLLA.set(limelightMeasurementCam1.pose);
      estTimeStamp.set(limelightMeasurementCam1.timestampSeconds);
    }

    // if our angular velocity is greater than 360 degrees per second, ignore vision updates
    // if(Math.abs(m_swerveDriveTrain.getPigeon2().getAngularVelocityZDevice().getValueAsDouble()) >
    // 720)
    // {
    //   doRejectUpdateLLA = true;
    // }
    if (limelightMeasurementCam1.tagCount == 0) {
      doRejectUpdateLLA = true;
    }
    if (limelightMeasurementCam1.tagCount >= 1) {
      doRejectUpdateLLA = false;
    }
    if (!doRejectUpdateLLA) {
      m_swerveDriveTrain.addVisionMeasurement(
          limelightMeasurementCam1.pose, limelightMeasurementCam1.timestampSeconds);
    }

    // limelight b
    LimelightHelpers.SetRobotOrientation(
        "limelight-b",
        m_swerveDriveTrain.getState().Pose.getRotation().getDegrees(),
        0,
        0,
        0,
        0,
        0);
    LimelightHelpers.PoseEstimate limelightMeasurementCam2 =
        LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2("limelight-b");
    m_swerveDriveTrain.setVisionMeasurementStdDevs(VecBuilder.fill(.5, .5, 9999999));

    if (limelightMeasurementCam2.timestampSeconds == 0) {
      DriverStation.reportWarning("LimelightB not running pose estimation", true);
    } else {
      DriverStation.reportWarning("LimelightB got vision pose", false);
      estPoseLLB.set(limelightMeasurementCam2.pose);
      estTimeStamp.set(limelightMeasurementCam2.timestampSeconds);
    }

    // // if our angular velocity is greater than 360 degrees per second, ignore vision updates
    // if(Math.abs(m_swerveDriveTrain.getPigeon2().getAngularVelocityZWorld().getValueAsDouble()) >
    // 720)
    // {
    //   doRejectUpdateLLB = true;
    // }
    if (limelightMeasurementCam2.tagCount == 0) {
      doRejectUpdateLLB = true;
    }
    if (limelightMeasurementCam2.tagCount >= 1) {
      doRejectUpdateLLA = false;
    }
    if (!doRejectUpdateLLB) {
      m_swerveDriveTrain.addVisionMeasurement(
          limelightMeasurementCam2.pose, limelightMeasurementCam2.timestampSeconds);
    }
    updateSmartDashboard();
  }

  @Override
  public void simulationPeriodic() {
    // if (m_swerveDriveTrain != null) {
    //   visionSim.update(m_swerveDriveTrain.getState().Pose);
    //   visionSim.getDebugField().setRobotPose(m_swerveDriveTrain.getState().Pose);
    // }
  }
}
