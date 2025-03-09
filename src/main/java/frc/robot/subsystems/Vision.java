package frc.robot.subsystems;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.net.PortForwarder;
import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructPublisher;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.LimelightHelpers;
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
      table.getStructTopic("estPoseLLA", Pose2d.struct).publish();
  private final DoublePublisher estTimeStamp = table.getDoubleTopic("estTimeStamp").publish();
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

  public void processLimelight(String limelightName) {
    LimelightHelpers.SetIMUMode(limelightName, 1);
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
      DriverStation.reportWarning(limelightName + " is not connected", true);
    } else {
      estPoseLLF.set(limelightMeasurement.pose);
      estTimeStamp.set(limelightMeasurement.timestampSeconds);

      if (limelightMeasurement.tagCount == 0) {
        doRejectUpdateLLF = true;
      }
      if (limelightMeasurement.tagCount >= 1) {
        doRejectUpdateLLF = false;
      }
      if (!doRejectUpdateLLF) {
        m_swerveDriveTrain.addVisionMeasurement(
            limelightMeasurement.pose, limelightMeasurement.timestampSeconds);
      }
    }
  }

  public void updateSmartDashboard() {
    SmartDashboard.putBoolean("rejectionUpdateLLF", doRejectUpdateLLF);
    SmartDashboard.putBoolean("rejectionUpdateLLB", doRejectUpdateLLB);
    SmartDashboard.putBoolean("trackingState", m_swerveDriveTrain.isTrackingState());
  }

  @Override
  public void periodic() {
    // limelight f
    processLimelight(CAMERA_SERVER.limelightF.toString());

    // limelight b
    processLimelight(CAMERA_SERVER.limelightB.toString());

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
