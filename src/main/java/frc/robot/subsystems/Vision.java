package frc.robot.subsystems;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.net.PortForwarder;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.ROBOT;
import frc.robot.constants.VISION;
// import frc.robot.simulation.FieldSim;
import java.util.List;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.simulation.PhotonCameraSim;
import org.photonvision.simulation.SimCameraProperties;
import org.photonvision.simulation.VisionSystemSim;
import org.photonvision.targeting.PhotonTrackedTarget;

public class Vision extends SubsystemBase {
  private CommandSwerveDrivetrain m_swerveDriveTrain;

  // private FieldSim m_fieldSim;
  private Translation2d m_goal = new Translation2d();

  private final NetworkTable NoteDetectionLimelight =
      NetworkTableInstance.getDefault().getTable("limelight");

  // public static final PhotonCamera aprilTagLimelightCameraA = new PhotonCamera("LimelightA");
  // PhotonPoseEstimator limelightPhotonPoseEstimatorA =
  //     new PhotonPoseEstimator(
  //         VISION.aprilTagFieldLayout,
  //         PhotonPoseEstimator.PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR,
  //         aprilTagLimelightCameraA,
  //         VISION.robotToAprilTagLimelightCameraA);

  public static final PhotonCamera aprilTagLimelightCameraB = new PhotonCamera("LimelightB");
  PhotonPoseEstimator limelightPhotonPoseEstimatorB =
      new PhotonPoseEstimator(
          VISION.aprilTagFieldLayout,
          PhotonPoseEstimator.PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR,
          VISION.robotToAprilTagLimelightCameraB);
  private Matrix<N3, N1> curStdDevs;
  private VisionSystemSim visionSim;
  // private PhotonCameraSim aprilTagLimelightCameraASim;
  private PhotonCameraSim aprilTagLimelightCameraBSim;

  // private Pose2d cameraAEstimatedPose = new Pose2d();
  private Pose2d cameraBEstimatedPose = new Pose2d();
  private double /*cameraATimestamp,*/ cameraBTimestamp;
  private boolean cameraAHasPose, cameraBHasPose, poseAgreement;
  private boolean m_localized;

  public Vision() {

    // Port Forwarding to access limelight on USB Ethernet
    for (int port = 5800; port <= 5807; port++) {
      PortForwarder.add(port, VISION.CAMERA_SERVER.INTAKE.toString(), port);
    }

    PortForwarder.add(5800, VISION.CAMERA_SERVER.LIMELIGHTB.toString(), 5800);

    limelightPhotonPoseEstimatorB.setMultiTagFallbackStrategy(
        PhotonPoseEstimator.PoseStrategy.CLOSEST_TO_REFERENCE_POSE);
    if (RobotBase.isSimulation()) {
      // Create the vision system simulation which handles cameras and targets on the field.
      visionSim = new VisionSystemSim("main");
      // Add all the AprilTags inside the tag layout as visible targets to this simulated field.
      visionSim.addAprilTags(VISION.aprilTagFieldLayout);
      // Create simulated camera properties. These can be set to mimic your actual camera.
      var cameraProp = new SimCameraProperties();
      cameraProp.setCalibration(960, 720, Rotation2d.fromDegrees(VISION.kLimelightDFOV));
      cameraProp.setCalibError(0.35, 0.10);
      cameraProp.setFPS(45);
      cameraProp.setAvgLatencyMs(100);
      cameraProp.setLatencyStdDevMs(15);
      // Create a PhotonCameraSim which will update the linked PhotonCamera's values with visible
      // targets.
      // aprilTagLimelightCameraASim = new PhotonCameraSim(aprilTagLimelightCameraA, cameraProp);
      aprilTagLimelightCameraBSim = new PhotonCameraSim(aprilTagLimelightCameraB, cameraProp);
      // Add the simulated camera to view the targets on this simulated field.
      // visionSim.addCamera(aprilTagLimelightCameraASim, VISION.robotToAprilTagLimelightCameraA);
      visionSim.addCamera(aprilTagLimelightCameraBSim, VISION.robotToAprilTagLimelightCameraB);

      // aprilTagLimelightCameraASim.enableDrawWireframe(false);
      aprilTagLimelightCameraBSim.enableDrawWireframe(false);
    }
  }

  public void registerSwerveDrive(CommandSwerveDrivetrain swerveDriveTrain) {
    m_swerveDriveTrain = swerveDriveTrain;
  }

  // public void registerFieldSim(FieldSim fieldSim) {
  //   m_fieldSim = fieldSim;
  // }

  public boolean checkPoseAgreement(Pose3d a, Pose3d b) {
    var poseDelta = a.minus(b);

    if (Math.abs(poseDelta.getTranslation().getX()) > VISION.poseXTolerance) {
      return false;
    }

    if (Math.abs(poseDelta.getTranslation().getY()) > VISION.poseYTolerance) {
      return false;
    }

    //    if (Math.abs(poseDelta.getTranslation().getZ()) > VISION.poseZTolerance) {
    //      return false;
    //    }

    //    if (Math.abs(poseDelta.getRotation().getX()) > VISION.poseRollTolerance) {
    //      return false;
    //    }
    //
    //    if (Math.abs(poseDelta.getRotation().getY()) > VISION.posePitchTolerance) {
    //      return false;
    //    }

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

  public boolean hasGamePieceTarget() {
    NetworkTableEntry tv = NoteDetectionLimelight.getEntry("tv");
    return tv.getDouble(0.0) == 1;
  }

  public double getRobotToGamePieceDegrees() {
    double degreesRotation = 0.0;
    if (hasGamePieceTarget()) {
      NetworkTableEntry tx = NoteDetectionLimelight.getEntry("tx");
      degreesRotation = tx.getDouble(0.0);
    }
    return degreesRotation;
  }

  public Rotation2d getRobotToGamePieceRotation() {
    return Rotation2d.fromDegrees(getRobotToGamePieceDegrees());
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
    // Implement the smartDashboard method here
  }

  private void updateLog() {}

  @Override
  public void periodic() {
    if (DriverStation.isDisabled()) {
      if (cameraBHasPose) m_localized = true;
    }
    if (m_swerveDriveTrain != null && !DriverStation.isAutonomous()) {
      // final var globalPoseA = getEstimatedGlobalPose(limelightPhotonPoseEstimatorA);
      // globalPoseA.ifPresentOrElse(
      //     estimatedRobotPose -> {
      //       cameraAEstimatedPose = estimatedRobotPose.estimatedPose;
      //       cameraATimestamp = estimatedRobotPose.timestampSeconds;
      //       cameraAHasPose = true;
      //     },
      //     () -> {
      //       cameraAEstimatedPose = nullPose;
      //       cameraAHasPose = false;
      //     });
      var change = aprilTagLimelightCameraB.getLatestResult();
      limelightPhotonPoseEstimatorB.setReferencePose(m_swerveDriveTrain.getState().Pose);
      limelightPhotonPoseEstimatorB
          .update(change)
          .ifPresent(
              (estimatedRobotPose) -> {
                cameraBEstimatedPose = estimatedRobotPose.estimatedPose.toPose2d();
                cameraBTimestamp = estimatedRobotPose.timestampSeconds;
                cameraBHasPose = true;
                m_swerveDriveTrain.addVisionMeasurement(cameraBEstimatedPose, cameraBTimestamp);
              });
      //      final var globalPoseB = getEstimatedGlobalPose(limelightPhotonPoseEstimatorB);
      //      globalPoseB.ifPresent(
      //          (estimatedRobotPose) -> {
      //            cameraBEstimatedPose = estimatedRobotPose.estimatedPose.toPose2d();
      //            cameraBTimestamp = estimatedRobotPose.timestampSeconds;
      //            cameraBHasPose = true;
      //            m_swerveDriveTrain.addVisionMeasurement(cameraBEstimatedPose, cameraBTimestamp);
      //          });

      // if (cameraAHasPose && cameraBHasPose) {
      //   poseAgreement = checkPoseAgreement(cameraAEstimatedPose, cameraBEstimatedPose);

      // if (poseAgreement) {
      //   m_swerveDriveTrain.addVisionMeasurement(
      //       cameraAEstimatedPose.toPose2d(), cameraATimestamp);
      //   m_swerveDriveTrain.addVisionMeasurement(
      //       cameraBEstimatedPose.toPose2d(), cameraBTimestamp);
      // }

      // if (getTargetAmount(aprilTagLimelightCameraB) >= 2) {
      //   m_swerveDriveTrain.addVisionMeasurement(
      //       cameraBEstimatedPose, cameraBTimestamp);
      // }
    }

    // updateAngleToNote();
    // This method will be called once per scheduler run
    updateSmartDashboard();
    if (ROBOT.logMode.get() <= ROBOT.LOG_MODE.NORMAL.get()) updateLog();

    // if (m_fieldSim != null) {
    //   // m_fieldSim.updateVisionAPose(cameraAEstimatedPose);
    //   m_fieldSim.updateVisionBPose(cameraBEstimatedPose);
    // }
  }

  @Override
  public void simulationPeriodic() {
    if (m_swerveDriveTrain != null) {
      visionSim.update(m_swerveDriveTrain.getState().Pose);
      visionSim.getDebugField().setRobotPose(m_swerveDriveTrain.getState().Pose);
    }
  }
}
