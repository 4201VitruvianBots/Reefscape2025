package frc.robot.constants;

import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Meters;
import static frc.robot.constants.SWERVE.kBumperThickness;
import static frc.robot.constants.SWERVE.kWheelBase;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.*;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj.DriverStation;
import java.util.Arrays;
import java.util.HashMap;
import java.util.Map;
import java.util.stream.Stream;

public class FIELD {
  /**
   * FIELD
   *
   * <p>Field constants
   *
   * <p>Note: Values are using ideal values from WPILib TODO: Create layout from practice field.
   */
  public static final AprilTagFieldLayout wpilibAprilTagLayout =
      AprilTagFieldLayout.loadField(AprilTagFields.k2025ReefscapeWelded);

  // public static final AprilTagFieldLayout practiceFieldAprilTagLayout;

  public static final AprilTagFieldLayout aprilTagFieldLayout = wpilibAprilTagLayout;

  /** Field X-axis */
  public static final Distance LENGTH = Meters.of(aprilTagFieldLayout.getFieldLength());

  /** Field Y-axis */
  public static final Distance WIDTH = Meters.of(aprilTagFieldLayout.getFieldWidth());

  public static final Distance APRILTAG_SIZE = Inches.of(6.5);

  public static final Distance CORAL_STATION_HEIGHT =
      Inches.of(55.25 + APRILTAG_SIZE.in(Inches) / 2.0);
  public static final Distance PROCESSOR_HEIGHT =
      Inches.of(47.875 + APRILTAG_SIZE.in(Inches) / 2.0);
  public static final Distance BARGE_HEIGHT = Inches.of(70.75 + APRILTAG_SIZE.in(Inches) / 2.0);
  public static final Distance REEF_HEIGHT = Inches.of(8.75 + APRILTAG_SIZE.in(Inches) / 2.0);

  /** Enum describing all AprilTags on the field by ID and their Pose3d positions. */
  public enum APRIL_TAG {
    RED_CORAL_STATION_LEFT(1),
    RED_CORAL_STATION_RIGHT(2),
    RED_PROCESSOR(3),
    BLUE_BARGE_FAR(4),
    RED_BARGE_NEAR(5),
    RED_REEF_NEAR_LEFT(6),
    RED_REEF_NEAR_CENTER(7),
    RED_REEF_NEAR_RIGHT(8),
    RED_REEF_FAR_RIGHT(9),
    RED_REEF_FAR_CENTER(10),
    RED_REEF_FAR_LEFT(11),
    BLUE_CORAL_STATION_RIGHT(12),
    BLUE_CORAL_STATION_LEFT(13),
    BLUE_BARGE_NEAR(14),
    RED_BARGE_FAR(15),
    BLUE_PROCESSOR(16),
    BLUE_REEF_NEAR_RIGHT(17),
    BLUE_REEF_NEAR_CENTER(18),
    BLUE_REEF_NEAR_LEFT(19),
    BLUE_REEF_FAR_LEFT(20),
    BLUE_REEF_FAR_CENTER(21),
    BLUE_REEF_FAR_RIGHT(22);

    private final int id;
    private Pose3d pose;

    APRIL_TAG(final int id) {
      this.id = id;
      aprilTagFieldLayout
          .getTagPose(this.id)
          .ifPresentOrElse(
              pose3d -> this.pose = pose3d,
              () -> {
                System.out.printf(
                    "[FIELD] Could not read AprilTag ID %s data from FieldLayout\n", this.id);
                new Alert(
                    String.format("[FIELD] APRIL_TAG ID %s value couldn't be read", this.id),
                    AlertType.kError);
              });

      if (this.pose == null) {
        throw new IllegalArgumentException("AprilTag ID " + this.id + " does not have a Pose3d!");
      }
    }

    public int getId() {
      return id;
    }

    public Pose3d getPose3d() {
      return pose;
    }

    public Pose2d getPose2d() {
      return pose.toPose2d();
    }

    public Translation3d getTranslation3d() {
      return pose.getTranslation();
    }

    public Translation2d getTranslation2d() {
      return getPose2d().getTranslation();
    }

    public static APRIL_TAG getTagById(int aprilTagId) {
      for (var t : APRIL_TAG.values()) {
        if (t.id == aprilTagId) {
          return t;
        }
      }
      throw new IllegalArgumentException("AprilTag ID " + aprilTagId + " does not exist!");
    }

    public static Pose2d[] getAllAprilTagPoses() {
      return Arrays.stream(APRIL_TAG.values()).map(APRIL_TAG::getPose2d).toArray(Pose2d[]::new);
    }
  }

  // TODO: Is this still needed?
  //  public enum APRIL_TAG_OFFSETS {
  //    RED_REEF_NEAR_LEFT(6),
  //    RED_REEF_NEAR_CENTER(7),
  //    RED_REEF_NEAR_RIGHT(8),
  //    RED_REEF_FAR_RIGHT(9),
  //    RED_REEF_FAR_CENTER(10),
  //    RED_REEF_FAR_LEFT(11),
  //    BLUE_REEF_NEAR_RIGHT(17),
  //    BLUE_REEF_NEAR_CENTER(18),
  //    BLUE_REEF_NEAR_LEFT(19),
  //    BLUE_REEF_FAR_LEFT(20),
  //    BLUE_REEF_FAR_CENTER(21),
  //    BLUE_REEF_FAR_RIGHT(22);
  //
  //
  //    APRIL_TAG_OFFSETS(final int aprilTagId, final Angle angleOffset, final Distance ) {
  //
  //    }
  //  }

  /** Depth of the reef poles from the AprilTag */
  //  static Distance baseReefDepth = Inches.of(30.738);  // Mechanical Advantage's measurement
  //  static Distance baseReefDepth = Inches.of(24); // CAD Measurement to base
  static final Distance baseReefDepth = Inches.of(2); // CAD Measurement L4 CC distance

  /**
   * Left/right translation of the reef poles perpendicular to the center of the AprilTag's Pose2d.
   */
  static final Distance baseReefTranslation = Inches.of(6.469);

  static final Translation2d leftReefOffset =
      new Translation2d(-baseReefDepth.in(Meters), baseReefTranslation.in(Meters));
  static final Translation2d rightReefOffset =
      new Translation2d(-baseReefDepth.in(Meters), -baseReefTranslation.in(Meters));

  static final Translation2d baseLeftBranchTargetOffset =
      new Translation2d(
          kWheelBase.plus(kBumperThickness).in(Meters), baseReefTranslation.in(Meters));
  static final Translation2d baseRightBranchTargetOffset =
      new Translation2d(
          kWheelBase.plus(kBumperThickness).in(Meters), -baseReefTranslation.in(Meters));

  static Translation2d adjustableLeftBranchOffset = new Translation2d();
  static Translation2d adjustableRightBranchOffset = new Translation2d();

  /**
   * Location of all Reef Branches based on their position relative to their nearest AprilTag.
   * Includes the ability to add a left/right offset in inches to account for field differences
   */
  public enum REEF_BRANCHES {
    RED_REEF_NEAR_LEFT_LEFT(6, true, Inches.of(0)),
    RED_REEF_NEAR_LEFT_RIGHT(6, false, Inches.of(0)),
    RED_REEF_NEAR_CENTER_LEFT(7, true, Inches.of(0)),
    RED_REEF_NEAR_CENTER_RIGHT(7, false, Inches.of(0)),
    RED_REEF_NEAR_RIGHT_LEFT(8, true, Inches.of(0)),
    RED_REEF_NEAR_RIGHT_RIGHT(8, false, Inches.of(0)),
    RED_REEF_FAR_RIGHT_LEFT(9, true, Inches.of(0)),
    RED_REEF_FAR_RIGHT_RIGHT(9, false, Inches.of(0)),
    RED_REEF_FAR_CENTER_LEFT(10, true, Inches.of(0)),
    RED_REEF_FAR_CENTER_RIGHT(10, false, Inches.of(0)),
    RED_REEF_FAR_LEFT_LEFT(11, true, Inches.of(0)),
    RED_REEF_FAR_LEFT_RIGHT(11, false, Inches.of(0)),
    BLUE_REEF_NEAR_RIGHT_LEFT(17, true, Inches.of(0)),
    BLUE_REEF_NEAR_RIGHT_RIGHT(17, false, Inches.of(0)),
    BLUE_REEF_NEAR_CENTER_LEFT(18, true, Inches.of(0)),
    BLUE_REEF_NEAR_CENTER_RIGHT(18, false, Inches.of(0)),
    BLUE_REEF_NEAR_LEFT_LEFT(19, true, Inches.of(0)),
    BLUE_REEF_NEAR_LEFT_RIGHT(19, false, Inches.of(0)),
    BLUE_REEF_FAR_LEFT_LEFT(20, true, Inches.of(0)),
    BLUE_REEF_FAR_LEFT_RIGHT(20, false, Inches.of(0)),
    BLUE_REEF_FAR_CENTER_LEFT(21, true, Inches.of(0)),
    BLUE_REEF_FAR_CENTER_RIGHT(21, false, Inches.of(0)),
    BLUE_REEF_FAR_RIGHT_LEFT(22, true, Inches.of(0)),
    BLUE_REEF_FAR_RIGHT_RIGHT(22, false, Inches.of(0));

    private final Pose2d pose;

    // Pose2d we want the robot to go to relative to the branch's location
    private final Pose2d targetPose;

    private static final Map<Pose2d, Pose2d> branchPoseToTargetPose = new HashMap<>();

    REEF_BRANCHES(final int aprilTagId, boolean isLeft, Distance offset) {
      Pose2d branchPose = Pose2d.kZero;
      Pose2d targetPose = Pose2d.kZero;

      var branchOffset = isLeft ? leftReefOffset : rightReefOffset;
      branchOffset.plus(new Translation2d(0, offset.in(Meters)));
      var targetOffset = isLeft ? baseLeftBranchTargetOffset : baseRightBranchTargetOffset;
      try {
        branchPose =
            APRIL_TAG
                .getTagById(aprilTagId)
                .getPose2d()
                .plus(new Transform2d(branchOffset, Rotation2d.kZero));
      } catch (Exception e) {
        System.out.println(
            "[FIELD] Could not generate pose for reef at AprilTag "
                + aprilTagId
                + (isLeft ? " Left" : " Right"));
      }
      try {
        targetPose =
            APRIL_TAG
                .getTagById(aprilTagId)
                .getPose2d()
                .plus(new Transform2d(targetOffset, Rotation2d.kZero));

      } catch (Exception e) {
        System.out.println(
            "[FIELD] Could not generate targetPose for reef at AprilTag "
                + aprilTagId
                + (isLeft ? " Left" : " Right"));
      }

      pose = branchPose;
      this.targetPose = targetPose;
    }

    static {
      for (REEF_BRANCHES b : REEF_BRANCHES.values()) {
        branchPoseToTargetPose.put(b.getPose2d(), b.getTargetPose2d());
      }
    }

    /** Return the selected branch's position as a Pose2d */
    public Pose2d getPose2d() {
      return pose;
    }

    public Pose2d getTargetPose2d() {
      return targetPose;
    }

    /** 2d array of all branches */
    public static Pose2d[] getAllPose2d() {
      return Arrays.stream(REEF_BRANCHES.values())
          .map(REEF_BRANCHES::getPose2d)
          .toArray(Pose2d[]::new);
    }

    /** 2d array of branches by Alliance color */
    public static Pose2d[] getAlliancePose2d(DriverStation.Alliance alliance) {
      if (alliance == DriverStation.Alliance.Red) {
        return Arrays.stream(getAllPose2d()).limit(12).toArray(Pose2d[]::new);
      } else {
        return Arrays.stream(getAllPose2d()).skip(12).limit(12).toArray(Pose2d[]::new);
      }
    }

    /** 2d array of all branch targets */
    public static Pose2d[] getAllTargetPose2d() {
      return Arrays.stream(REEF_BRANCHES.values())
          .map(REEF_BRANCHES::getTargetPose2d)
          .toArray(Pose2d[]::new);
    }

    /** 2d array of branch targets by Alliance color */
    public static Pose2d[] getAllianceTargetPose2d(DriverStation.Alliance alliance) {
      if (alliance == DriverStation.Alliance.Red) {
        return Arrays.stream(getAllTargetPose2d()).limit(12).toArray(Pose2d[]::new);
      } else {
        return Arrays.stream(getAllTargetPose2d()).skip(12).limit(12).toArray(Pose2d[]::new);
      }
    }

    public static Pose2d getBranchPoseToTargetPose(Pose2d branchPose) {
      if (!branchPoseToTargetPose.containsKey(branchPose)) {
        DriverStation.reportWarning("[FIELD] Trying to use a non-existent branch pose!", false);
        return Pose2d.kZero;
      } else {
        return branchPoseToTargetPose.get(branchPose);
      }
    }
  }

  /** Static array of all Red Alliance Reef Branches to avoid constantly generating it */
  public static final Pose2d[] RED_BRANCHES =
      REEF_BRANCHES.getAlliancePose2d(DriverStation.Alliance.Red);

  /** Static array of all Blue Alliance Reef Branches to avoid constantly generating it */
  public static final Pose2d[] BLUE_BRANCHES =
      REEF_BRANCHES.getAlliancePose2d(DriverStation.Alliance.Blue);

  /** Static array of all Red Alliance Reef Branch targets to avoid constantly generating it */
  public static final Pose2d[] RED_BRANCH_TARGETS =
      REEF_BRANCHES.getAllianceTargetPose2d(DriverStation.Alliance.Red);

  /** Static array of all Blue Alliance Reef Branch targets to avoid constantly generating it */
  public static final Pose2d[] BLUE_BRANCH_TARGETS =
      REEF_BRANCHES.getAllianceTargetPose2d(DriverStation.Alliance.Blue);

  /** Position of each reef center */
  public static final Translation2d redReefCenter =
      new Translation2d(LENGTH.minus(Inches.of(176.746)).in(Meters), Inches.of(158.501).in(Meters));

  public static final Translation2d blueReefCenter =
      new Translation2d(Inches.of(176.746).in(Meters), Inches.of(158.501).in(Meters));

  /** Calculation of Reef Zones */
  public static final Distance reefZoneDepth = Inches.of(120);

  public static final Distance reefZoneWidth = Inches.of(120);
  public static final Translation2d reefZoneCornerA =
      new Translation2d(reefZoneDepth.in(Meters), -reefZoneWidth.in(Meters) / 2.0);
  public static final Translation2d reefZoneCornerB =
      new Translation2d(reefZoneDepth.in(Meters), reefZoneWidth.in(Meters) / 2.0);

  /**
   * Zones are polygons defined by Pose2d points. Reef Zones are defined as a triangle with three
   * points starting from the center of a reef, a point projected from the center of the reef
   * perpendicular to a reef face by the distance reefZoneDepth, and a third point projected 90
   * degrees by half the distance defined by reefZoneWidth (left/right defines if this projection is
   * positive/negative from the second point)
   */
  public enum ZONES {
    RED_REEF_NEAR_LEFT_LEFT(6, true),
    RED_REEF_NEAR_LEFT_RIGHT(6, false),
    RED_REEF_NEAR_CENTER_LEFT(7, true),
    RED_REEF_NEAR_CENTER_RIGHT(7, false),
    RED_REEF_NEAR_RIGHT_LEFT(8, true),
    RED_REEF_NEAR_RIGHT_RIGHT(8, false),
    RED_REEF_FAR_RIGHT_LEFT(9, true),
    RED_REEF_FAR_RIGHT_RIGHT(9, false),
    RED_REEF_FAR_CENTER_LEFT(10, true),
    RED_REEF_FAR_CENTER_RIGHT(10, false),
    RED_REEF_FAR_LEFT_LEFT(11, true),
    RED_REEF_FAR_LEFT_RIGHT(11, false),
    BLUE_REEF_NEAR_RIGHT_LEFT(17, true),
    BLUE_REEF_NEAR_RIGHT_RIGHT(17, false),
    BLUE_REEF_NEAR_CENTER_LEFT(18, true),
    BLUE_REEF_NEAR_CENTER_RIGHT(18, false),
    BLUE_REEF_NEAR_LEFT_LEFT(19, true),
    BLUE_REEF_NEAR_LEFT_RIGHT(19, false),
    BLUE_REEF_FAR_LEFT_LEFT(20, true),
    BLUE_REEF_FAR_LEFT_RIGHT(20, false),
    BLUE_REEF_FAR_CENTER_LEFT(21, true),
    BLUE_REEF_FAR_CENTER_RIGHT(21, false),
    BLUE_REEF_FAR_RIGHT_LEFT(22, true),
    BLUE_REEF_FAR_RIGHT_RIGHT(22, false);

    private final Pose2d[] corners;

    ZONES(int aprilTagId, boolean isLeft) {
      var aprilTagRotation = APRIL_TAG.getTagById(aprilTagId).getPose2d().getRotation();
      Pose2d cornerCenter =
          new Pose2d(aprilTagId < 12 ? redReefCenter : blueReefCenter, aprilTagRotation);
      corners =
          new Pose2d[] {
            cornerCenter,
            cornerCenter.plus(new Transform2d(reefZoneDepth.in(Meters), 0, Rotation2d.kZero)),
            cornerCenter.plus(
                new Transform2d(isLeft ? reefZoneCornerA : reefZoneCornerB, Rotation2d.kZero))
          };
    }

    /** Return an array of Pose2d of the defined Zone */
    public Pose2d[] getZone() {
      return corners;
    }

    /** 2d array of all defined zones */
    public static Pose2d[][] getAllZones() {
      return Arrays.stream(ZONES.values()).map(ZONES::getZone).toArray(Pose2d[][]::new);
    }

    /** 2d array of zones by Alliance color */
    public static Pose2d[][] getAllianceZones(DriverStation.Alliance alliance) {
      if (alliance == DriverStation.Alliance.Red) {
        return Arrays.stream(getAllZones()).limit(12).toArray(Pose2d[][]::new);
      } else {
        return Arrays.stream(getAllZones()).skip(12).limit(12).toArray(Pose2d[][]::new);
      }
    }
  }

  /** Static array of all Red Alliance Zones to avoid constantly generating it */
  public static final Pose2d[] RED_ZONES =
      Arrays.stream(ZONES.getAllianceZones(DriverStation.Alliance.Red))
          .flatMap(Stream::of)
          .toArray(Pose2d[]::new);

  /** Static array of all Blue Alliance Zones to avoid constantly generating it */
  public static final Pose2d[] BLUE_ZONES =
      Arrays.stream(ZONES.getAllianceZones(DriverStation.Alliance.Blue))
          .flatMap(Stream::of)
          .toArray(Pose2d[]::new);
}
