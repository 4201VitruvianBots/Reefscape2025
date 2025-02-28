package frc.robot.subsystems;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.net.PortForwarder;
import edu.wpi.first.networktables.DoubleArrayPublisher;
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

  private final NetworkTable m_limeLightA;
  private final NetworkTable m_limeLightB;

  private final DoubleArrayPublisher m_limelightAPositionPub;
  private final DoubleArrayPublisher m_limelightBPositionPub;

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

  // public static final PhotonCamera aprilTagLimelightCameraA = new PhotonCamera("LimelightA");
  // PhotonPoseEstimator limelightPhotonPoseEstimatorA =
  //     new PhotonPoseEstimator(
  //         VISION.aprilTagFieldLayout,
  //         PhotonPoseEstimator.PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR,
  //         VISION.robotToAprilTagLimelightCameraA);

  // public static final PhotonCamera aprilTagLimelightCameraB = new PhotonCamera("LimelightB");
  // PhotonPoseEstimator limelightPhotonPoseEstimatorB =
  //     new PhotonPoseEstimator(
  //         VISION.aprilTagFieldLayout,
  //         PhotonPoseEstimator.PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR,
  //         VISION.robotToAprilTagLimelightCameraB);

  private Matrix<N3, N1> curStdDevs;
  private VisionSystemSim visionSim;
  private PhotonCameraSim aprilTagLimelightCameraASim;
  private PhotonCameraSim aprilTagLimelightCameraBSim;

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
  private final NetworkTable table = inst.getTable("VisionDebug");
  private final DoublePublisher visionEstPoseTimestamp =
      table.getDoubleTopic("EstPoseTimestamp").publish();
  private final StructPublisher<Pose2d> visionEstPose =
      table.getStructTopic("EstPose", Pose2d.struct).publish();

  public Vision() {
    m_limeLightA = NetworkTableInstance.getDefault().getTable("limeLight-a");
    m_limeLightB = NetworkTableInstance.getDefault().getTable("limeLight-b");
    // TODO: Decide if this is necessary
    // Port Forwarding to access limelight on USB Ethernet
    for (int port = 5800; port <= 5809; port++) {
      PortForwarder.add(port, VISION.CAMERA_SERVER.LIMELIGHTA.toString(), port);
    }

    for (int port = 5800; port <= 5809; port++) {
      PortForwarder.add(port + 10, VISION.CAMERA_SERVER.LIMELIGHTB.toString(), port);
    }

    // PortForwarder.add(5800, VISION.CAMERA_SERVER.LIMELIGHTB.toString(), 5800);

    m_limelightAPositionPub =
        NetworkTableInstance.getDefault()
            .getTable("fLocalizer")
            .getDoubleArrayTopic("camToRobotT3D")
            .publish();

    m_limelightBPositionPub =
        NetworkTableInstance.getDefault()
            .getTable("bLocalizer")
            .getDoubleArrayTopic("camToRobotT3D")
            .publish();

    // Change the camera pose relative to robot center (x forward, y left, z up, degrees)
    LimelightHelpers.setCameraPose_RobotSpace(
        "limelight-a",
        0.3556, // Forward offset (meters)
        0.127, // Side offset (meters)
        0.2159, // Height offset (meters)
        0.0, // Roll (degrees)
        0.0, // Pitch (degrees)
        0.0); // Yaw (degrees)

    LimelightHelpers.setCameraPose_RobotSpace(
        "limelight-b",
        -0.3175, // Forward offset (meters)
        0.29845, // Side offset (meters)
        0.4318, // Height offset (meters)
        0.0, // Roll (degrees)
        5.0, // Pitch (degrees)
        0.0 // Yaw (degrees)
        );
    // limelightPhotonPoseEstimatorA.setMultiTagFallbackStrategy(
    //   PhotonPoseEstimator.PoseStrategy.CLOSEST_TO_REFERENCE_POSE);
    // limelightPhotonPoseEstimatorB.setMultiTagFallbackStrategy(
    //     PhotonPoseEstimator.PoseStrategy.CLOSEST_TO_REFERENCE_POSE);
    // if (RobotBase.isSimulation()) {
    //   // Create the vision system simulation which handles cameras and targets on the field.
    //   visionSim = new VisionSystemSim("main");
    //   // Add all the AprilTags inside the tag layout as visible targets to this simulated field.
    //   visionSim.addAprilTags(VISION.aprilTagFieldLayout);
    //   // Create simulated camera properties. These can be set to mimic your actual camera.
    //   var cameraProp = new SimCameraProperties();
    //   cameraProp.setCalibration(960, 720, Rotation2d.fromDegrees(VISION.kLimelightDFOV));
    //   cameraProp.setCalibError(0.35, 0.10);
    //   cameraProp.setFPS(45);
    //   cameraProp.setAvgLatencyMs(100);
    //   cameraProp.setLatencyStdDevMs(15);
    //   // Create a PhotonCameraSim which will update the linked PhotonCamera's values with visible
    //   // targets.
    //   aprilTagLimelightCameraASim = new PhotonCameraSim(aprilTagLimelightCameraA, cameraProp);
    //   aprilTagLimelightCameraBSim = new PhotonCameraSim(aprilTagLimelightCameraB, cameraProp);
    //   // Add the simulated camera to view the targets on this simulated field.
    //   visionSim.addCamera(aprilTagLimelightCameraASim, VISION.robotToAprilTagLimelightCameraA);
    //   visionSim.addCamera(aprilTagLimelightCameraBSim, VISION.robotToAprilTagLimelightCameraB);

    //   // aprilTagLimelightCameraASim.enableDrawWireframe(false);
    //   aprilTagLimelightCameraBSim.enableDrawWireframe(false);
    // }
  }

  public void registerSwerveDrive(CommandSwerveDrivetrain swerveDriveTrain) {
    m_swerveDriveTrain = swerveDriveTrain;
  }

  // public Optional<EstimatedRobotPose> getEstimatedGlobalPose() {
  //   Optional<EstimatedRobotPose> visionEst = Optional.empty();
  //   for (var change : aprilTagLimelightCameraB.getAllUnreadResults()) {
  //     visionEst = limelightPhotonPoseEstimatorB.update(change);
  //     updateEstimationStdDevs(visionEst, change.getTargets());

  //     if (Robot.isSimulation()) {
  //       visionEst.ifPresentOrElse(
  //           est ->
  //               getSimDebugField()
  //                   .getObject("VisionEstimation")
  //                   .setPose(est.estimatedPose.toPose2d()),
  //           () -> {
  //             getSimDebugField().getObject("VisionEstimation").setPoses();
  //           });
  //     }
  //   }
  //   return visionEst;
  // }

  public static final Matrix<N3, N1> kSingleTagStdDevs = VecBuilder.fill(4, 4, 8);
  public static final Matrix<N3, N1> kMultiTagStdDevs = VecBuilder.fill(0.5, 0.5, 1);

  // /**
  //  * Calculates new standard deviations This algorithm is a heuristic that creates dynamic
  // standard
  //  * deviations based on number of tags, estimation strategy, and distance from the tags.
  //  *
  //  * @param estimatedPose The estimated pose to guess standard deviations for.
  //  * @param targets All targets in this camera frame
  //  */
  // private void updateEstimationStdDevs(
  //     Optional<EstimatedRobotPose> estimatedPose, List<PhotonTrackedTarget> targets) {
  //   if (estimatedPose.isEmpty()) {
  //     // No pose input. Default to single-tag std devs
  //     curStdDevs = kSingleTagStdDevs;

  //   } else {
  //     // Pose present. Start running Heuristic
  //     var estStdDevs = kSingleTagStdDevs;
  //     int numTags = 0;
  //     double avgDist = 0;

  //     // Precalculation - see how many tags we found, and calculate an average-distance metric
  //     for (var tgt : targets) {
  //       var tagPose =
  // limelightPhotonPoseEstimatorB.getFieldTags().getTagPose(tgt.getFiducialId());
  //       if (tagPose.isEmpty()) continue;
  //       numTags++;
  //       avgDist +=
  //           tagPose
  //               .get()
  //               .toPose2d()
  //               .getTranslation()
  //               .getDistance(estimatedPose.get().estimatedPose.toPose2d().getTranslation());
  //     }

  //     if (numTags == 0) {
  //       // No tags visible. Default to single-tag std devs
  //       curStdDevs = kSingleTagStdDevs;
  //     } else {
  //       // One or more tags visible, run the full heuristic.
  //       avgDist /= numTags;
  //       // Decrease std devs if multiple targets are visible
  //       if (numTags > 1) estStdDevs = kMultiTagStdDevs;
  //       // Increase std devs based on (average) distance
  //       if (numTags == 1 && avgDist > 4)
  //         estStdDevs = VecBuilder.fill(Double.MAX_VALUE, Double.MAX_VALUE, Double.MAX_VALUE);
  //       else estStdDevs = estStdDevs.times(1 + (avgDist * avgDist / 30));
  //       curStdDevs = estStdDevs;
  //     }
  //   }
  // }

  // public Matrix<N3, N1> getEstimationStdDevs() {
  //   return curStdDevs;
  // }

  public void registerFieldSim(FieldSim fieldSim) {
    m_fieldSim = fieldSim;
  }

  public void setTrackingState(VISION.TRACKING_STATE state) {
    trackingState = state;
  }

  public boolean isCameraConnected(PhotonCamera camera) {
    return camera.isConnected();
  }

  // public boolean isAprilTagDetected(PhotonCamera camera) {
  //   // TODO: PhotonCamera.getLatestResult() is depreciated
  //   var result = camera.getLatestResult();
  //   return result.hasTargets();
  // }

  // public String getTargets(PhotonCamera camera) {
  //   // TODO: PhotonCamera.getLatestResult() is depreciated
  //   var result = camera.getLatestResult();
  //   List<PhotonTrackedTarget> targets = result.getTargets();
  //   return String.join(" ", targets.stream().map(PhotonTrackedTarget::toString).toList());
  // }

  // public int getTargetAmount(PhotonCamera camera) {
  //   // TODO: PhotonCamera.getLatestResult() is depreciated
  //   var result = camera.getLatestResult();
  //   List<PhotonTrackedTarget> targets = result.getTargets();
  //   return targets.size();
  // }

  // private void updateAngleToReef() {
  //     if (m_swerveDriveTrain != null) {
  //       if (DriverStation.isTeleop()) {
  //         m_goal = Controls.isRedAlliance() ? FIELD.redReefCenter : FIELD.blueReefCenter;

  //         double PositionY = m_swerveDriveTrain.getState().Pose.getY();
  //         double PositionX = m_swerveDriveTrain.getState().Pose.getX();
  //         double VelocityY = m_swerveDriveTrain.getChassisSpeed().vyMetersPerSecond;
  //         double VelocityX = m_swerveDriveTrain.getChassisSpeed().vxMetersPerSecond;
  //         double AccelerationX =
  // m_swerveDriveTrain.getPigeon2().getAccelerationX().getValueAsDouble();
  //         double AccelerationY =
  // m_swerveDriveTrain.getPigeon2().getAccelerationY().getValueAsDouble();
  //         double virtualGoalX = m_goal.getX() - VISION.velocityShoot * (VelocityX +
  // AccelerationX); // TODO: velocityShoot needs to be checked
  //         double virtualGoalY = m_goal.getY() - VISION.velocityShoot * (VelocityY +
  // AccelerationY);
  //         Translation2d movingGoalLocation = new Translation2d(virtualGoalX, virtualGoalY);
  //         Translation2d currentPose = m_swerveDriveTrain.getState().Pose.getTranslation();
  //         double newDist = movingGoalLocation.minus(currentPose).getDistance(new
  // Translation2d());

  //         m_swerveDriveTrain.setAngleToReef(
  //             m_swerveDriveTrain
  //                 .getState()
  //                 .Pose
  //                 .getTranslation()
  //                 .minus(m_goal)
  //                 .getAngle()
  //                 .plus(
  //                     Rotation2d.fromRadians(
  //                         Math.asin(
  //                             (((VelocityY * 0.85) * PositionX + (VelocityX * 0.2) * PositionY))
  //                                 / (newDist * 5)))));
  //       }
  //     }
  //   }

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
    SmartDashboard.putBoolean("limelightA_TV", limelightBHasTarget);
  }

  private void updateLog() {}

  @Override
  public void periodic() {
    if (DriverStation.isDisabled()) {
      if (cameraAHasPose || cameraBHasPose) m_localized = true;
    }
    if (m_swerveDriveTrain != null) {
      // limelightPhotonPoseEstimatorA.setReferencePose(m_swerveDriveTrain.getState().Pose);
      // var visionEstLimelightA = getEstimatedGlobalPose();
      // visionEstLimelightA.ifPresent(
      //     est -> {
      //       visionEstPose.set(est.estimatedPose);
      //       visionEstPoseTimestamp.set(est.timestampSeconds);

      //       // Change our trust in the measurement based on the tags we can see
      //       var estStdDevs = getEstimationStdDevs();

      //       m_swerveDriveTrain.addVisionMeasurement(
      //           est.estimatedPose.toPose2d(), est.timestampSeconds, estStdDevs);
      //     });

      // limelighta setup
      try {
        if (useMegaTag2 == false) {
          LimelightHelpers.PoseEstimate mt1_limelightA =
              LimelightHelpers.getBotPoseEstimate_wpiBlue("limelight-a");
          if (mt1_limelightA.tagCount == 1 && mt1_limelightA.rawFiducials.length == 1) {
            if (mt1_limelightA.rawFiducials[0].ambiguity > .7) {
              doRejectUpdate = true;
            }
            if (mt1_limelightA.rawFiducials[0].distToCamera > 3) {
              doRejectUpdate = true;
            }
          }
          if (mt1_limelightA.tagCount == 0) {
            doRejectUpdate = true;
          }
          if (!doRejectUpdate) {
            visionEstPose.set(mt1_limelightA.pose);
            visionEstPoseTimestamp.set(mt1_limelightA.timestampSeconds);
            m_swerveDriveTrain.setVisionMeasurementStdDevs(VecBuilder.fill(.5, .5, 9999999));
            m_swerveDriveTrain.addVisionMeasurement(
                mt1_limelightA.pose, mt1_limelightA.timestampSeconds);
          } else if (useMegaTag2 == true) {
            LimelightHelpers.SetRobotOrientation(
                "limelight-a",
                m_swerveDriveTrain.getState().Pose.getRotation().getDegrees(),
                0,
                0,
                0,
                0,
                0);
            if (DriverStation.isAutonomous()) {
              LimelightHelpers.SetIMUMode("limelight-a", 1);
            } else {
              LimelightHelpers.SetIMUMode("limelight-a", 2);
            }
            LimelightHelpers.PoseEstimate mt2_limelightA =
                LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2("limelight-a");

            // if our angular velocity is greater than 720 degrees per second, ignore vision updates
            if (Math.abs(
                    m_swerveDriveTrain.getPigeon2().getAngularVelocityZDevice().getValueAsDouble())
                > 720) {
              doRejectUpdate = true;
            }
            if (mt2_limelightA.tagCount == 0) {
              doRejectUpdate = true;
            }
            if (!doRejectUpdate) {
              visionEstPose.set(mt2_limelightA.pose);
              visionEstPoseTimestamp.set(mt2_limelightA.timestampSeconds);
              m_swerveDriveTrain.setVisionMeasurementStdDevs(VecBuilder.fill(.7, .7, 9999999));
              m_swerveDriveTrain.addVisionMeasurement(
                  mt2_limelightA.pose, mt2_limelightA.timestampSeconds);
            }
          }
        }
      } catch (NullPointerException e) {
        DriverStation.reportWarning(
            "LimelightA failed to get pose esstimation from NetworkTables", true);
      }

      // limelightb setup
      try {
        if (useMegaTag2 == false) {
          LimelightHelpers.PoseEstimate mt1_limelightB =
              LimelightHelpers.getBotPoseEstimate_wpiBlue("limelight-a");
          if (mt1_limelightB.tagCount == 1 && mt1_limelightB.rawFiducials.length == 1) {
            if (mt1_limelightB.rawFiducials[0].ambiguity > .7) {
              doRejectUpdate = true;
            }
            if (mt1_limelightB.rawFiducials[0].distToCamera > 3) {
              doRejectUpdate = true;
            }
          }
          if (mt1_limelightB.tagCount == 0) {
            doRejectUpdate = true;
          }
          if (!doRejectUpdate) {
            visionEstPose.set(mt1_limelightB.pose);
            visionEstPoseTimestamp.set(mt1_limelightB.timestampSeconds);
            m_swerveDriveTrain.setVisionMeasurementStdDevs(VecBuilder.fill(.5, .5, 9999999));
            m_swerveDriveTrain.addVisionMeasurement(
                mt1_limelightB.pose, mt1_limelightB.timestampSeconds);
          } else if (useMegaTag2 == true) {
            LimelightHelpers.SetRobotOrientation(
                "limelight-b",
                m_swerveDriveTrain.getState().Pose.getRotation().getDegrees(),
                0,
                0,
                0,
                0,
                0);
            if (DriverStation.isAutonomous()) {
              LimelightHelpers.SetIMUMode("limelight-a", 1);
            } else {
              LimelightHelpers.SetIMUMode("limelight-a", 2);
            }
            LimelightHelpers.PoseEstimate mt2_limelightB =
                LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2("limelight-b");

            // if our angular velocity is greater than 720 degrees per second, ignore vision updates
            if (Math.abs(
                    m_swerveDriveTrain.getPigeon2().getAngularVelocityZDevice().getValueAsDouble())
                > 720) {
              doRejectUpdate = true;
            }
            if (mt2_limelightB.tagCount == 0) {
              doRejectUpdate = true;
            }
            if (!doRejectUpdate) {
              visionEstPose.set(mt2_limelightB.pose);
              visionEstPoseTimestamp.set(mt2_limelightB.timestampSeconds);
              m_swerveDriveTrain.setVisionMeasurementStdDevs(VecBuilder.fill(.7, .7, 9999999));
              m_swerveDriveTrain.addVisionMeasurement(
                  mt2_limelightB.pose, mt2_limelightB.timestampSeconds);
            }
          }
        }
      } catch (NullPointerException e) {
        DriverStation.reportWarning(
            "LimelightB failed to get pose esstimation from NetworkTables", true);
      }

      // LimelightHelpers.SetRobotOrientation("limelight-b",
      // m_swerveDriveTrain.getState().Pose.getRotation().getDegrees(), 0, 0, 0, 0, 0);
      // LimelightHelpers.PoseEstimate mt2_limelightB =
      // LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2("limelight-b");
      // if(Math.abs(m_swerveDriveTrain.getPigeon2().getAngularVelocityZDevice().getValueAsDouble())
      // > 720) // if our angular velocity is greater than 720 degrees per second, ignore vision
      // updates
      // {
      //   doRejectUpdate = true;
      // }
      // if(mt2_limelightB.tagCount == 0)
      // {
      //   doRejectUpdate = true;
      // }
      // if(!doRejectUpdate)
      // {
      //   visionEstPose.set(mt2_limelightA.pose);
      //   visionEstPoseTimestamp.set(mt2_limelightA.timestampSeconds);
      //   m_swerveDriveTrain.setVisionMeasurementStdDevs(VecBuilder.fill(.7,.7,9999999));
      //   m_swerveDriveTrain.addVisionMeasurement(
      //     mt2_limelightB.pose,
      //     mt2_limelightB.timestampSeconds);
      // }

      // Correct pose estimate with vision measurements
      // limelightPhotonPoseEstimatorB.setReferencePose(m_swerveDriveTrain.getState().Pose);
      // var visionEstLimelightB = getEstimatedGlobalPose();
      // visionEstLimelightB.ifPresent(
      //     est -> {
      //       visionEstPose.set(est.estimatedPose);
      //       visionEstPoseTimestamp.set(est.timestampSeconds);

      //       // Change our trust in the measurement based on the tags we can see
      //       var estStdDevs = getEstimationStdDevs();

      //       m_swerveDriveTrain.addVisionMeasurement(
      //           est.estimatedPose.toPose2d(), est.timestampSeconds, estStdDevs);
      //     });

      // // if (cameraAHasPose && cameraBHasPose) {
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

    switch (trackingState) {
      case BRANCH:
        updateAngleToBranch();
        break;
      case NONE:
      default:
        break;
    }

    m_limelightAPositionPub.set(
        new double[] {
          VISION.LOCALIZER_CAMERA_POSITION[0].getTranslation().getX(),
          VISION.LOCALIZER_CAMERA_POSITION[0].getTranslation().getY(),
          VISION.LOCALIZER_CAMERA_POSITION[0].getTranslation().getZ(),
        });

    m_limelightBPositionPub.set(
        new double[] {
          VISION.LOCALIZER_CAMERA_POSITION[1].getTranslation().getX(),
          VISION.LOCALIZER_CAMERA_POSITION[1].getTranslation().getY(),
          VISION.LOCALIZER_CAMERA_POSITION[1].getTranslation().getZ(),
        });

    // This method will be called once per scheduler run
    updateSmartDashboard();
    if (ROBOT.logMode.get() <= ROBOT.LOG_MODE.NORMAL.get()) updateLog();

    if (m_fieldSim != null) {
      m_fieldSim.addPoses("CameraAEstimatedPose", cameraAEstimatedPose);
      m_fieldSim.addPoses("CameraBEstimatedPose", cameraBEstimatedPose);
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
