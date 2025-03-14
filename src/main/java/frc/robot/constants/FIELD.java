package frc.robot.constants;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.*;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj.DriverStation;
import java.util.Arrays;
import java.util.HashMap;
import java.util.List;
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

  public static final Distance CORAL_STATION_HEIGHT = Inches.of(55.25).plus(APRILTAG_SIZE).div(2.0);
  public static final Distance PROCESSOR_HEIGHT = Inches.of(47.875).plus(APRILTAG_SIZE).div(2.0);
  public static final Distance BARGE_HEIGHT = Inches.of(70.75).plus(APRILTAG_SIZE).div(2.0);
  public static final Distance REEF_HEIGHT = Inches.of(8.75).plus(APRILTAG_SIZE).div(2.0);

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
                        AlertType.kError)
                    .set(true);
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

  //
  // Coral Branch constants
  //

  /** Depth of the reef poles from the AprilTag */
  //  static Distance baseReefDepth = Inches.of(30.738);  // Mechanical Advantage's measurement
  //  static Distance baseReefDepth = Inches.of(24); // CAD Measurement to base
  static final Distance baseReefDepth = Inches.of(2); // CAD Measurement L4 CC distance

  // TODO: Verify these offset is correct
  /**
   * Left/right translation of the reef poles perpendicular to the center of the AprilTag's Pose2d.
   */
  static final Distance baseReefTranslation = Inches.of(8);

  static final Translation2d leftReefCoralOffset =
      new Translation2d(-baseReefDepth.in(Meters), baseReefTranslation.in(Meters));
  static final Translation2d rightReefCoralOffset =
      new Translation2d(-baseReefDepth.in(Meters), -baseReefTranslation.in(Meters));

  static final Translation2d baseLeftCoralTargetOffset =
      new Translation2d(
          SWERVE.kWheelBase.div(2).plus(SWERVE.kBumperThickness).minus(Inches.of(1.5)).in(Meters),
          baseReefTranslation.in(Meters));
  static final Translation2d baseRightCoralTargetOffset =
      new Translation2d(
          SWERVE.kWheelBase.div(2).plus(SWERVE.kBumperThickness).minus(Inches.of(1.5)).in(Meters),
          -baseReefTranslation.in(Meters));

  /**
   * Location of all coral targets based on their position relative to their nearest AprilTag.
   * Includes the ability to add a left/right offset in inches to account for field differences
   */
  public enum CORAL_TARGETS {
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

    /** Pose2d of the coral */
    private final Pose2d pose;

    /** Pose2d we want the robot to go to relative to the branch's location */
    private final Pose2d targetPose;

    /** If the branch is a left branch (When facing the Reef) */
    private final boolean isLeftBranch;

    /** Map to convert the coral's Pose2d to the target Pose2d */
    private static final Map<Pose2d, Pose2d> coralPoseToTargetPose = new HashMap<>();

    CORAL_TARGETS(final int aprilTagId, boolean isLeft, Distance offset) {
      Pose2d branchPose = Pose2d.kZero;
      Pose2d targetPose = Pose2d.kZero;

      var branchOffset = isLeft ? leftReefCoralOffset : rightReefCoralOffset;
      branchOffset.plus(new Translation2d(0, offset.in(Meters)));
      var targetOffset = isLeft ? baseLeftCoralTargetOffset : baseRightCoralTargetOffset;
      try {
        var aprilTagPose = APRIL_TAG.getTagById(aprilTagId).getPose2d();
        branchPose = aprilTagPose.plus(new Transform2d(branchOffset, Rotation2d.kZero));

        targetPose = aprilTagPose.plus(new Transform2d(targetOffset, Rotation2d.kZero));
        targetPose = new Pose2d(targetPose.getTranslation(), targetPose.getRotation().plus(Rotation2d.k180deg));
      } catch (Exception e) {
        System.out.printf(
            "[FIELD] Could not get AprilTag %d Pose for CORAL_TARGET generation!\n", aprilTagId);
      }
      this.pose = branchPose;
      this.targetPose = targetPose;
      this.isLeftBranch = isLeft;
    }

    /** Map the coral pose to the target pose */
    static {
      for (CORAL_TARGETS b : CORAL_TARGETS.values()) {
        coralPoseToTargetPose.put(b.getPose2d(), b.getTargetPose2d());
      }
    }

    /** Return the selected coral's position as a Pose2d */
    public Pose2d getPose2d() {
      return pose;
    }

    /** Return the selected coral position's target as a Pose2d */
    public Pose2d getTargetPose2d() {
      return targetPose;
    }

    public boolean isLeftBranch() {
      return isLeftBranch;
    }

    /** 2d array of all coral positions */
    public static Pose2d[] getAllPose2d() {
      return Arrays.stream(CORAL_TARGETS.values())
          .map(CORAL_TARGETS::getPose2d)
          .toArray(Pose2d[]::new);
    }

    public static Pose2d[] getAllLeftPose2d() {
      return Arrays.stream(CORAL_TARGETS.values())
          .filter(CORAL_TARGETS::isLeftBranch)
          .map(CORAL_TARGETS::getPose2d)
          .toArray(Pose2d[]::new);
    }

    public static Pose2d[] getAllRightPose2d() {
      return Arrays.stream(CORAL_TARGETS.values())
          .filter(c -> !c.isLeftBranch())
          .map(CORAL_TARGETS::getPose2d)
          .toArray(Pose2d[]::new);
    }

    /** 2d array of coral positions by Alliance color */
    public static Pose2d[] getAlliancePose2d(DriverStation.Alliance alliance) {
      if (alliance == DriverStation.Alliance.Red) {
        return Arrays.stream(getAllPose2d()).limit(12).toArray(Pose2d[]::new);
      } else {
        return Arrays.stream(getAllPose2d()).skip(12).limit(12).toArray(Pose2d[]::new);
      }
    }

    /** 2d array of left coral positions by Alliance color */
    public static Pose2d[] getAllianceLeftPose2d(DriverStation.Alliance alliance) {
      if (alliance == DriverStation.Alliance.Red) {
        return Arrays.stream(getAllLeftPose2d()).limit(6).toArray(Pose2d[]::new);
      } else {
        return Arrays.stream(getAllLeftPose2d()).skip(6).limit(6).toArray(Pose2d[]::new);
      }
    }

    /** 2d array of right coral positions by Alliance color */
    public static Pose2d[] getAllianceRightPose2d(DriverStation.Alliance alliance) {
      if (alliance == DriverStation.Alliance.Red) {
        return Arrays.stream(getAllRightPose2d()).limit(6).toArray(Pose2d[]::new);
      } else {
        return Arrays.stream(getAllRightPose2d()).skip(6).limit(6).toArray(Pose2d[]::new);
      }
    }

    /** 2d array of all coral targets */
    public static Pose2d[] getAllTargetPose2d() {
      return Arrays.stream(CORAL_TARGETS.values())
          .map(CORAL_TARGETS::getTargetPose2d)
          .toArray(Pose2d[]::new);
    }

    /** 2d array of coral targets by Alliance color */
    public static Pose2d[] getAllianceTargetPose2d(DriverStation.Alliance alliance) {
      if (alliance == DriverStation.Alliance.Red) {
        return Arrays.stream(getAllTargetPose2d()).limit(12).toArray(Pose2d[]::new);
      } else {
        return Arrays.stream(getAllTargetPose2d()).skip(12).limit(12).toArray(Pose2d[]::new);
      }
    }

    /** Convert the nearest coral position to the target position */
    public static Pose2d getCoralPoseToTargetPose(Pose2d branchPose) {
      if (!coralPoseToTargetPose.containsKey(branchPose)) {
        DriverStation.reportWarning("[FIELD] Trying to use a non-existent coral pose!", false);
        return Pose2d.kZero;
      } else {
        return coralPoseToTargetPose.get(branchPose);
      }
    }
  }

  /** Static array of all Red Alliance Reef Branches to avoid constantly generating it */
  public static final List<Pose2d> RED_CORAL_BRANCHES =
      Arrays.asList(CORAL_TARGETS.getAlliancePose2d(DriverStation.Alliance.Red));

  /** Static array of all Blue Alliance Reef Branches to avoid constantly generating it */
  public static final List<Pose2d> BLUE_CORAL_BRANCHES =
      Arrays.asList(CORAL_TARGETS.getAlliancePose2d(DriverStation.Alliance.Blue));

  /** Static array of all Red Alliance Reef Branch targets to avoid constantly generating it */
  public static final List<Pose2d> RED_CORAL_TARGETS =
      Arrays.asList(CORAL_TARGETS.getAllianceTargetPose2d(DriverStation.Alliance.Red));

  /** Static array of all Blue Alliance Reef Branch targets to avoid constantly generating it */
  public static final List<Pose2d> BLUE_CORAL_TARGETS =
      Arrays.asList(CORAL_TARGETS.getAllianceTargetPose2d(DriverStation.Alliance.Blue));

  /** Static array of all Red Alliance Left Reef Branches to avoid constantly generating it */
  public static final List<Pose2d> RED_CORAL_LEFT_BRANCHES =
      Arrays.asList(CORAL_TARGETS.getAllianceLeftPose2d(DriverStation.Alliance.Red));

  /** Static array of all Red Alliance Right Reef Branches to avoid constantly generating it */
  public static final List<Pose2d> RED_CORAL_RIGHT_BRANCHES =
      Arrays.asList(CORAL_TARGETS.getAllianceRightPose2d(DriverStation.Alliance.Red));

  /** Static array of all Blue Alliance Left Reef Branches to avoid constantly generating it */
  public static final List<Pose2d> BLUE_CORAL_LEFT_BRANCHES =
      Arrays.asList(CORAL_TARGETS.getAllianceLeftPose2d(DriverStation.Alliance.Blue));

  /** Static array of all Blue Alliance Right Reef Branches to avoid constantly generating it */
  public static final List<Pose2d> BLUE_CORAL_RIGHT_BRANCHES =
      Arrays.asList(CORAL_TARGETS.getAllianceRightPose2d(DriverStation.Alliance.Blue));

  //
  // Algae Branch constants
  //

  // TODO: Verify this offset is correct
  static final Translation2d baseAlgaeTargetOffset =
      new Translation2d(SWERVE.kWheelBase.div(2).plus(SWERVE.kBumperThickness).in(Meters), 0);

  public enum TARGET_TYPE {
    REEF,
    PROCESSOR,
    BARGE,
    /** Ground Algae staged at the beginning of a match */
    STAGED
  }

  public enum ALGAE_TARGETS {
    RED_REEF_NEAR_LEFT(6, TARGET_TYPE.REEF, Inches.of(0)),
    RED_REEF_NEAR_CENTER(7, TARGET_TYPE.REEF, Inches.of(0)),
    RED_REEF_NEAR_RIGHT(8, TARGET_TYPE.REEF, Inches.of(0)),
    RED_REEF_FAR_RIGHT(9, TARGET_TYPE.REEF, Inches.of(0)),
    RED_REEF_FAR_CENTER(10, TARGET_TYPE.REEF, Inches.of(0)),
    RED_REEF_FAR_LEFT(11, TARGET_TYPE.REEF, Inches.of(0)),
    BLUE_REEF_NEAR_RIGHT(17, TARGET_TYPE.REEF, Inches.of(0)),
    BLUE_REEF_NEAR_CENTER(18, TARGET_TYPE.REEF, Inches.of(0)),
    BLUE_REEF_NEAR_LEFT(19, TARGET_TYPE.REEF, Inches.of(0)),
    BLUE_REEF_FAR_LEFT(20, TARGET_TYPE.REEF, Inches.of(0)),
    BLUE_REEF_FAR_CENTER(21, TARGET_TYPE.REEF, Inches.of(0)),
    BLUE_REEF_FAR_RIGHT(22, TARGET_TYPE.REEF, Inches.of(0));

    /** Pose2d of the algae */
    private final Pose2d pose;

    /** Pose2d we want the robot to go to relative to the algae location */
    private final Pose2d targetPose;

    /** Map to convert the algae Pose2d to the target Pose2d */
    private static final Map<Pose2d, Pose2d> algaePoseToTargetPose = new HashMap<>();

    ALGAE_TARGETS(final int aprilTagId, TARGET_TYPE type, Distance offset) {
      Pose2d aprilTagPose = Pose2d.kZero;

      try {
        aprilTagPose = APRIL_TAG.getTagById(aprilTagId).getPose2d();
      } catch (Exception e) {
        System.out.printf(
            "[FIELD] Could not get AprilTag %d Pose for ALGAE_TARGET generation!\n", aprilTagId);
      }

      // TODO: Implement
      //      Translation2d targetOffset = Translation2d.kZero;
      //      switch (type) {
      //        case REEF -> targetOffset = baseReefAlgaeTargetOffset;
      //        case PROCESSOR -> targetOffset = Translation2d.kZero;
      //        case BARGE -> targetOffset = Translation2d.kZero;
      //        case STAGED -> targetOffset = Translation2d.kZero;
      //      }

      Pose2d targetPose =
          aprilTagPose.plus(
              new Transform2d(
                  baseAlgaeTargetOffset.plus(new Translation2d(offset.in(Meters), 0)),
                  Rotation2d.kZero));

      this.pose = aprilTagPose;
      this.targetPose = targetPose;
    }

    /** Map the algae pose to the target pose */
    static {
      for (ALGAE_TARGETS b : ALGAE_TARGETS.values()) {
        algaePoseToTargetPose.put(b.getPose2d(), b.getTargetPose2d());
      }
    }

    /** Return the selected algae position as a Pose2d */
    public Pose2d getPose2d() {
      return pose;
    }

    public Pose2d getTargetPose2d() {
      return targetPose;
    }

    /** 2d array of all algae */
    public static Pose2d[] getAllPose2d() {
      return Arrays.stream(ALGAE_TARGETS.values())
          .map(ALGAE_TARGETS::getPose2d)
          .toArray(Pose2d[]::new);
    }

    /** 2d array of algae by Alliance color */
    public static Pose2d[] getAlliancePose2d(DriverStation.Alliance alliance) {
      if (alliance == DriverStation.Alliance.Red) {
        return Arrays.stream(getAllPose2d()).limit(6).toArray(Pose2d[]::new);
      } else {
        return Arrays.stream(getAllPose2d()).skip(6).limit(6).toArray(Pose2d[]::new);
      }
    }

    /** 2d array of all algae targets */
    public static Pose2d[] getAllTargetPose2d() {
      return Arrays.stream(ALGAE_TARGETS.values())
          .map(ALGAE_TARGETS::getTargetPose2d)
          .toArray(Pose2d[]::new);
    }

    /** 2d array of algae targets by Alliance color */
    public static Pose2d[] getAllianceTargetPose2d(DriverStation.Alliance alliance) {
      if (alliance == DriverStation.Alliance.Red) {
        return Arrays.stream(getAllTargetPose2d()).limit(6).toArray(Pose2d[]::new);
      } else {
        return Arrays.stream(getAllTargetPose2d()).skip(6).limit(6).toArray(Pose2d[]::new);
      }
    }

    /** Convert the nearest algae position to the target position */
    public static Pose2d getAlgaePoseToTargetPose(Pose2d branchPose) {
      if (!algaePoseToTargetPose.containsKey(branchPose)) {
        DriverStation.reportWarning("[FIELD] Trying to use a non-existent algae pose!", false);
        return Pose2d.kZero;
      } else {
        return algaePoseToTargetPose.get(branchPose);
      }
    }
  }

  /** Static array of all Red Alliance Reef Branches to avoid constantly generating it */
  public static final List<Pose2d> RED_ALGAE_BRANCHES =
      Arrays.asList(ALGAE_TARGETS.getAlliancePose2d(DriverStation.Alliance.Red));

  /** Static array of all Blue Alliance Reef Branches to avoid constantly generating it */
  public static final List<Pose2d> BLUE_ALGAE_BRANCHES =
      Arrays.asList(ALGAE_TARGETS.getAlliancePose2d(DriverStation.Alliance.Blue));

  /** Static array of all Red Alliance Reef Branch targets to avoid constantly generating it */
  public static final List<Pose2d> RED_ALGAE_TARGETS =
      Arrays.asList(ALGAE_TARGETS.getAllianceTargetPose2d(DriverStation.Alliance.Red));

  /** Static array of all Blue Alliance Reef Branch targets to avoid constantly generating it */
  public static final List<Pose2d> BLUE_ALGAE_TARGETS =
      Arrays.asList(ALGAE_TARGETS.getAllianceTargetPose2d(DriverStation.Alliance.Blue));

  //
  // Zone Constants
  //

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
