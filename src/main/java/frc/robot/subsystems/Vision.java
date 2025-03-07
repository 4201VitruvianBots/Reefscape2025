package frc.robot.subsystems;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.net.PortForwarder;
import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructPublisher;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.LimelightHelpers;
import frc.robot.Robot;
import frc.robot.constants.FIELD;
import frc.robot.constants.ROBOT;
import frc.robot.constants.VISION;
import java.util.Arrays;
// import frc.robot.simulation.FieldSim;
import org.photonvision.PhotonCamera;
import org.photonvision.simulation.PhotonCameraSim;
import org.photonvision.simulation.VisionSystemSim;
import org.team4201.codex.simulation.FieldSim;

public class Vision extends SubsystemBase {
  private CommandSwerveDrivetrain m_swerveDriveTrain;
  private FieldSim m_fieldSim;

  private NetworkTable limelightATable = NetworkTableInstance.getDefault().getTable("limelight-a");
  private NetworkTableEntry txOfLimelightA = limelightATable.getEntry("tx");
  private NetworkTableEntry tyOfLimelightA = limelightATable.getEntry("ty");
  private NetworkTableEntry taOfLimelightA = limelightATable.getEntry("ta");

  // read values periodically
  private double xOfLimelightA = txOfLimelightA.getDouble(0.0);
  private double yOfLimelightA = tyOfLimelightA.getDouble(0.0);
  private double areaOfLimelightA = taOfLimelightA.getDouble(0.0);
  private boolean limelightAHasTarget = LimelightHelpers.getTV("limelight-a");

  private NetworkTable limelightBTable = NetworkTableInstance.getDefault().getTable("limelight-b");
  private NetworkTableEntry txOfLimelightB = limelightBTable.getEntry("tx");
  private NetworkTableEntry tyOfLimelightB = limelightBTable.getEntry("ty");
  private NetworkTableEntry taOfLimelightB = limelightBTable.getEntry("ta");

  // read values periodically
  private double xOfLimelightB = txOfLimelightB.getDouble(0.0);
  private double yOfLimelightB = tyOfLimelightB.getDouble(0.0);
  private double areaOfLimelightB = taOfLimelightB.getDouble(0.0);
  private boolean limelightBHasTarget = LimelightHelpers.getTV("limelight-b");

  private VisionSystemSim visionSim;

  private Pose2d cameraAEstimatedPose = new Pose2d();
  private Pose2d cameraBEstimatedPose = new Pose2d();
  private double cameraATimestamp, cameraBTimestamp;
  private boolean cameraAHasPose, cameraBHasPose, poseAgreement;
  private boolean m_localized;
  private boolean doRejectUpdate = false;
  private boolean useMegaTag2 = true; // set to false to use MegaTag1
  private VISION.TRACKING_STATE trackingState = VISION.TRACKING_STATE.NONE;

  // NetworkTables publisher setup
  private final NetworkTableInstance inst = NetworkTableInstance.getDefault();

  /* Robot swerve drive state */
  private final NetworkTable table = inst.getTable("LimelightPoseEstimate");
  private final StructPublisher<Pose2d> estPoseLLA =
      table.getStructTopic("estPoseLLA", Pose2d.struct).publish();
  private final DoublePublisher estTimeStamp =
      table.getDoubleTopic("estTimeStamp").publish();
  private final StructPublisher<Pose2d> estPoseLLB =
      table.getStructTopic("estPoseLLB", Pose2d.struct).publish();

  public Vision() {
    // Port Forwarding to access limelight on USB Ethernet
    for (int port = 5800; port <= 5809; port++) {
      PortForwarder.add(port, VISION.CAMERA_SERVER.LIMELIGHTA.toString(), port);
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
    // limelight a target data
    SmartDashboard.putNumber("LimelightA_X", xOfLimelightA);
    SmartDashboard.putNumber("LimelightA_Y", yOfLimelightA);
    SmartDashboard.putNumber("LimelightA_Area", areaOfLimelightA);
    SmartDashboard.putBoolean("limelightA_TV", limelightAHasTarget);
    // limelight b target data
    SmartDashboard.putNumber("LimelightB_X", xOfLimelightB);
    SmartDashboard.putNumber("LimelightB_Y", yOfLimelightB);
    SmartDashboard.putNumber("LimelightB_Area", areaOfLimelightB);
    SmartDashboard.putBoolean("limelightB_TV", limelightBHasTarget);
  }

  private void updateLog() {}

  @Override
  public void periodic() {
    // limelight a
    LimelightHelpers.SetRobotOrientation("limelight-a", m_swerveDriveTrain.getState().Pose.getRotation().getDegrees(), 0, 0, 0, 0, 0);
    LimelightHelpers.PoseEstimate limelightMeasurementCam1 = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2("limelight-a");
    m_swerveDriveTrain.setVisionMeasurementStdDevs(VecBuilder.fill(.5, .5, 9999999));
    
    if(limelightMeasurementCam1.timestampSeconds == 0) {
      DriverStation.reportWarning("LimelightA not running pose estimation", true);
    }
    else {
      DriverStation.reportWarning("LimelightA got vison pose", false);
      estPoseLLA.set(limelightMeasurementCam1.pose);
      estTimeStamp.set(limelightMeasurementCam1.timestampSeconds);
      m_swerveDriveTrain.addVisionMeasurement(
        limelightMeasurementCam1.pose,
        limelightMeasurementCam1.timestampSeconds
      );
    }
  
    // limelight b
    LimelightHelpers.SetRobotOrientation("limelight-b", m_swerveDriveTrain.getState().Pose.getRotation().getDegrees(), 0, 0, 0, 0, 0);
    LimelightHelpers.PoseEstimate limelightMeasurementCam2 = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2("limelight-b");
    m_swerveDriveTrain.setVisionMeasurementStdDevs(VecBuilder.fill(.5, .5, 9999999));
    
    if(limelightMeasurementCam2.timestampSeconds == 0) {
      DriverStation.reportWarning("LimelightB not running pose estimation", true);
    }
    else {
      DriverStation.reportWarning("LimelightB got vison pose", false);
      estPoseLLB.set(limelightMeasurementCam2.pose);
      estTimeStamp.set(limelightMeasurementCam2.timestampSeconds);
      m_swerveDriveTrain.addVisionMeasurement(
        limelightMeasurementCam2.pose,
        limelightMeasurementCam2.timestampSeconds
      );
    }
  }

  @Override
  public void simulationPeriodic() {
    // if (m_swerveDriveTrain != null) {
    //   visionSim.update(m_swerveDriveTrain.getState().Pose);
    //   visionSim.getDebugField().setRobotPose(m_swerveDriveTrain.getState().Pose);
    // }
  }

  public Field2d getSimDebugField() {
    if (!Robot.isSimulation()) return null;
    return visionSim.getDebugField();
  }
}
