package frc.robot.subsystems;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.net.PortForwarder;
import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructPublisher;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Robot;
import frc.robot.constants.ROBOT;
import frc.robot.constants.VISION;
import java.util.List;
import java.util.Optional;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.simulation.PhotonCameraSim;
import org.photonvision.simulation.SimCameraProperties;
import org.photonvision.simulation.VisionSystemSim;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

public class Vision extends SubsystemBase {
  private CommandSwerveDrivetrain m_swerveDriveTrain;
  public static final PhotonCamera aprilTagLimelightCameraB = new PhotonCamera("LimelightB");
  PhotonPoseEstimator limelightPhotonPoseEstimatorB = new PhotonPoseEstimator(
      VISION.aprilTagFieldLayout,
      PhotonPoseEstimator.PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR,
      VISION.robotToAprilTagLimelightCameraB);

  private Matrix<N3, N1> curStdDevs;
  private VisionSystemSim visionSim;
  private PhotonCameraSim aprilTagLimelightCameraBSim;

  private Pose2d cameraBEstimatedPose = new Pose2d();
  private double cameraBTimestamp;
  private boolean cameraBHasPose;
  private boolean m_localized;

  // Networktables publisher setup
  private NetworkTableInstance inst = NetworkTableInstance.getDefault();
  private final NetworkTable table = inst.getTable("VisionDebug");
  private final DoublePublisher visionEstPoseTimestamp = table.getDoubleTopic("EstPoseTimestamp").publish();
  private final StructPublisher<Pose3d> visionEstPose = table.getStructTopic("EstPose", Pose3d.struct)
      .publish();

  public Vision() {
    // Port Forwarding to access limelight on USB Ethernet
    for (int port = 5800; port <= 5807; port++) {
      PortForwarder.add(port, VISION.CAMERA_SERVER.INTAKE.toString(), port);
    }

    PortForwarder.add(5800, VISION.CAMERA_SERVER.LIMELIGHTB.toString(), 5800);

    limelightPhotonPoseEstimatorB.setMultiTagFallbackStrategy(
        PhotonPoseEstimator.PoseStrategy.CLOSEST_TO_REFERENCE_POSE);

    if (RobotBase.isSimulation()) {
      // Create the vision system simulation which handles cameras and targets on the
      // field.
      visionSim = new VisionSystemSim("main");
      // Add all the AprilTags inside the tag layout as visible targets to this
      // simulated field.
      visionSim.addAprilTags(VISION.aprilTagFieldLayout);
      // Create simulated camera properties. These can be set to mimic your actual
      // camera.
      var cameraProp = new SimCameraProperties();
      cameraProp.setCalibration(960, 720, Rotation2d.fromDegrees(VISION.kLimelightDFOV));
      cameraProp.setCalibError(0.35, 0.10);
      cameraProp.setFPS(15);
      cameraProp.setAvgLatencyMs(100);
      cameraProp.setLatencyStdDevMs(15);
      // Create a PhotonCameraSim which will update the linked PhotonCamera's values
      // with visible
      // targets.
      aprilTagLimelightCameraBSim = new PhotonCameraSim(aprilTagLimelightCameraB, cameraProp);
      // Add the simulated camera to view the targets on this simulated field.
      visionSim.addCamera(aprilTagLimelightCameraBSim, VISION.robotToAprilTagLimelightCameraB);

      aprilTagLimelightCameraBSim.enableDrawWireframe(true);
    }
  }

  public void registerSwerveDrive(CommandSwerveDrivetrain swerveDriveTrain) {
    m_swerveDriveTrain = swerveDriveTrain;
  }

  public Optional<EstimatedRobotPose> getEstimatedGlobalPose() {
    Optional<EstimatedRobotPose> visionEst = Optional.empty();
    for (PhotonPipelineResult change : aprilTagLimelightCameraB.getAllUnreadResults()) {
      visionEst = limelightPhotonPoseEstimatorB.update(change);

      updateEstimationStdDevs(visionEst, change.getTargets());

      if (Robot.isSimulation()) {
        visionEst.ifPresentOrElse(
            est -> getSimDebugField()
                .getObject("VisionEstimation")
                .setPose(est.estimatedPose.toPose2d()),
            () -> {
              getSimDebugField().getObject("VisionEstimation").setPoses();
            });
      }
    }
    return visionEst;
  }

  public static final Matrix<N3, N1> kSingleTagStdDevs = VecBuilder.fill(4, 4, 8);
  public static final Matrix<N3, N1> kMultiTagStdDevs = VecBuilder.fill(0.5, 0.5, 1);

  /**
   * Calculates new standard deviations This algorithm is a heuristic that creates
   * dynamic standard
   * deviations based on number of tags, estimation strategy, and distance from
   * the tags.
   *
   * @param estimatedPose The estimated pose to guess standard deviations for.
   * @param targets       All targets in this camera frame
   */
  private void updateEstimationStdDevs(
      Optional<EstimatedRobotPose> estimatedPose, List<PhotonTrackedTarget> targets) {
    if (estimatedPose.isEmpty()) {
      // No pose input. Default to single-tag std devs
      curStdDevs = kSingleTagStdDevs;

    } else {
      // Pose present. Start running Heuristic
      var estStdDevs = kSingleTagStdDevs;
      int numTags = 0;
      double avgDist = 0;

      // Precalculation - see how many tags we found, and calculate an
      // average-distance metric
      for (var tgt : targets) {
        var tagPose = limelightPhotonPoseEstimatorB.getFieldTags().getTagPose(tgt.getFiducialId());
        if (tagPose.isEmpty())
          continue;
        numTags++;
        avgDist += tagPose
            .get()
            .toPose2d()
            .getTranslation()
            .getDistance(estimatedPose.get().estimatedPose.toPose2d().getTranslation());
      }

      if (numTags == 0) {
        // No tags visible. Default to single-tag std devs
        curStdDevs = kSingleTagStdDevs;
      } else {
        // One or more tags visible, run the full heuristic.
        avgDist /= numTags;
        // Decrease std devs if multiple targets are visible
        if (numTags > 1)
          estStdDevs = kMultiTagStdDevs;
        // Increase std devs based on (average) distance
        if (numTags == 1 && avgDist > 4)
          estStdDevs = VecBuilder.fill(Double.MAX_VALUE, Double.MAX_VALUE, Double.MAX_VALUE);
        else
          estStdDevs = estStdDevs.times(1 + (avgDist * avgDist / 30));
        curStdDevs = estStdDevs;
      }
    }
  }

  public Matrix<N3, N1> getEstimationStdDevs() {
    return curStdDevs;
  }

  // public void registerFieldSim(FieldSim fieldSim) {
  // m_fieldSim = fieldSim;
  // }

  public boolean checkPoseAgreement(Pose3d a, Pose3d b) {
    var poseDelta = a.minus(b);

    if (Math.abs(poseDelta.getTranslation().getX()) > VISION.poseXTolerance) {
      return false;
    }

    if (Math.abs(poseDelta.getTranslation().getY()) > VISION.poseYTolerance) {
      return false;
    }

    // if (Math.abs(poseDelta.getTranslation().getZ()) > VISION.poseZTolerance) {
    // return false;
    // }

    // if (Math.abs(poseDelta.getRotation().getX()) > VISION.poseRollTolerance) {
    // return false;
    // }
    //
    // if (Math.abs(poseDelta.getRotation().getY()) > VISION.posePitchTolerance) {
    // return false;
    // }

    if (Math.abs(poseDelta.getRotation().getZ()) > VISION.poseYawTolerance) {
      return false;
    }

    return true;
  }

  public boolean isCameraConnected(PhotonCamera camera) {
    return camera.isConnected();
  }

  public boolean isAprilTagDetected(PhotonCamera camera) {
    var result = camera.getLatestResult();
    return result.hasTargets();
  }

  public String getTargets(PhotonCamera camera) {
    var result = camera.getLatestResult();
    List<PhotonTrackedTarget> targets = result.getTargets();
    return String.join(" ", targets.stream().map(PhotonTrackedTarget::toString).toList());
  }

  public int getTargetAmount(PhotonCamera camera) {
    var result = camera.getLatestResult();
    List<PhotonTrackedTarget> targets = result.getTargets();
    return targets.size();
  }

  public boolean getInitialLocalization() {
    return m_localized;
  }

  public void resetInitialLocalization() {
    m_localized = false;
  }

  private void updateSmartDashboard() {
  }

  private void updateLog() {
  }

  @Override
  public void periodic() {
    if (DriverStation.isDisabled()) {
      if (cameraBHasPose)
        m_localized = true;
    }
    if (m_swerveDriveTrain != null && !DriverStation.isAutonomous()) {
      // Correct pose estimate with vision measurements
      var visionEst = getEstimatedGlobalPose();
      visionEst.ifPresent(
          est -> {
            // Change our trust in the measurement based on the tags we can see
            var estStdDevs = getEstimationStdDevs();

            visionEstPose.set(est.estimatedPose);
            visionEstPoseTimestamp.set(est.timestampSeconds);

            m_swerveDriveTrain.addVisionMeasurement(
                est.estimatedPose.toPose2d(), est.timestampSeconds, estStdDevs);
          });

      updateSmartDashboard();

      if (ROBOT.logMode.get() <= ROBOT.LOG_MODE.NORMAL.get())
        updateLog();
    }
  }

  @Override
  public void simulationPeriodic() {
    if (m_swerveDriveTrain != null && Robot.isSimulation()) {
      visionSim.update(m_swerveDriveTrain.getState().Pose);
      visionSim.getDebugField().setRobotPose(m_swerveDriveTrain.getState().Pose);
    }
  }

  public Field2d getSimDebugField() {
    if (!Robot.isSimulation())
      return null;
    return visionSim.getDebugField();
  }
}
