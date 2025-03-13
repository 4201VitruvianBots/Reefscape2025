package frc.robot.utils;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import frc.robot.LimelightHelpers;
import frc.robot.constants.FIELD;
import frc.robot.constants.VISION;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import java.util.Optional;
import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.simulation.PhotonCameraSim;
import org.photonvision.simulation.SimCameraProperties;
import org.photonvision.simulation.VisionSystemSim;

/** Use Photonlib to simulate Limelights */
public class LimelightSim {
  CommandSwerveDrivetrain m_swerveDrive;

  private PhotonPoseEstimator photonEstimatorLlf;
  private PhotonPoseEstimator photonEstimatorLLb;
  private PhotonCamera limelightF;
  private PhotonCamera limelightB;

  private VisionSystemSim visionSim;

  private PhotonCameraSim llfSim;
  private PhotonCameraSim llbSim;

  public LimelightSim(CommandSwerveDrivetrain swerveDrive) {
    m_swerveDrive = swerveDrive;

    limelightF = new PhotonCamera(VISION.CAMERA_SERVER.limelightF.toString());
    limelightB = new PhotonCamera(VISION.CAMERA_SERVER.limelightB.toString());
    photonEstimatorLlf =
        new PhotonPoseEstimator(
            FIELD.aprilTagFieldLayout,
            PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR,
            VISION.limelightFPosition);
    photonEstimatorLlf.setMultiTagFallbackStrategy(
        PhotonPoseEstimator.PoseStrategy.LOWEST_AMBIGUITY);
    photonEstimatorLLb =
        new PhotonPoseEstimator(
            FIELD.aprilTagFieldLayout,
            PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR,
            VISION.limelightFPosition);
    photonEstimatorLLb.setMultiTagFallbackStrategy(
        PhotonPoseEstimator.PoseStrategy.LOWEST_AMBIGUITY);

    visionSim = new VisionSystemSim("limelightSim");
    visionSim.addAprilTags(FIELD.aprilTagFieldLayout);

    var cameraProp = new SimCameraProperties();
    cameraProp.setCalibration(1280, 960, new Rotation2d(VISION.kLimelight4DFOV));
    cameraProp.setFPS(45);
    // TODO: Check these values
    cameraProp.setCalibError(0.35, 0.10);
    cameraProp.setAvgLatencyMs(50);
    cameraProp.setLatencyStdDevMs(15);
    llfSim = new PhotonCameraSim(limelightF, cameraProp);
    llbSim = new PhotonCameraSim(limelightB, cameraProp);
    visionSim.addCamera(llfSim, VISION.limelightFPosition);
    visionSim.addCamera(llfSim, VISION.limelightBPosition);

    llfSim.enableDrawWireframe(true);
    llbSim.enableDrawWireframe(true);
  }

  public LimelightHelpers.PoseEstimate getSimulatedResultsLimelightF() {
    //        LimelightHelpers.PoseEstimate results = null;
    LimelightHelpers.PoseEstimate results = new LimelightHelpers.PoseEstimate();
    Optional<EstimatedRobotPose> visionEst = Optional.empty();
    for (var change : limelightF.getAllUnreadResults()) {
      visionEst = photonEstimatorLlf.update(change);
      change.metadata.getLatencyMillis();

      if (RobotBase.isSimulation()) {
        visionEst.ifPresentOrElse(
            est ->
                getSimDebugField()
                    .getObject("VisionEstimationLLf")
                    .setPose(est.estimatedPose.toPose2d()),
            () -> {
              getSimDebugField().getObject("VisionEstimationLLf").setPoses();
            });
      }
    }
    // TODO: WIP
    //        visionEst.ifPresent(p->{
    //            results = new LimelightHelpers.PoseEstimate(
    //                p.estimatedPose.toPose2d(),
    //                p.timestampSeconds,
    //                p.
    //            );
    //            results.botpose_wpiblue = new double[]{
    //                    p.estimatedPose.getX(),
    //                    p.estimatedPose.getY(),
    //                    p.estimatedPose.getZ(),
    //                    p.estimatedPose.getRotation().getMeasureX().in(Degrees),
    //                    p.estimatedPose.getRotation().getMeasureY().in(Degrees),
    //                    p.estimatedPose.getRotation().getMeasureZ().in(Degrees),
    //            };
    //            results.
    //            if(DriverStation.)
    //        });
    //
    return results;
  }

  public void simulationPeriodic() {
    visionSim.update(m_swerveDrive.getState().Pose);
  }

  public void resetSimPose(Pose2d pose) {
    if (RobotBase.isSimulation()) visionSim.resetRobotPose(pose);
  }

  public Field2d getSimDebugField() {
    if (!RobotBase.isSimulation()) return null;
    return visionSim.getDebugField();
  }
}
