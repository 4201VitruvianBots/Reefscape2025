package frc.robot.constants;

import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Meters;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.units.measure.Distance;

public class FIELD {
  /**
   * FIELD
   *
   * <p>Field constants
   *
   * <p>Note: Values are using ideal values from WPILib TODO: Create layout from practice field.
   */
  public static final AprilTagFieldLayout wpilibAprilTagLayout =
      AprilTagFieldLayout.loadField(AprilTagFields.k2024Crescendo);

  //    public static final AprilTagFieldLayout practiceFieldAprilTagLayout;

  public static AprilTagFieldLayout aprilTagFieldLayout = wpilibAprilTagLayout;

  // Field X-axis
  public static final Distance LENGTH = Meters.of(aprilTagFieldLayout.getFieldLength());
  // Field Y-axis
  public static final Distance WIDTH = Meters.of(aprilTagFieldLayout.getFieldWidth());

  public static final Distance APRILTAG_SIZE = Inches.of(6.5);

  public static final Distance CORAL_STATION_HEIGHT =
      Inches.of(55.25 + APRILTAG_SIZE.in(Inches) / 2.0);
  public static final Distance PROCESSOR_HEIGHT =
      Inches.of(47.875 + APRILTAG_SIZE.in(Inches) / 2.0);
  public static final Distance BARGE_HEIGHT = Inches.of(70.75 + APRILTAG_SIZE.in(Inches) / 2.0);
  public static final Distance REEF_HEIGHT = Inches.of(8.75 + APRILTAG_SIZE.in(Inches) / 2.0);

  public enum APRIL_TAG {
    RED_CORAL_STATION_LEFT(1),
    RED_CORAL_STATION_RIGHT(2),
    RED_PROCESSOR(3),
    BLUE_BARGE_FAR(4),
    RED_BARGE_NEAR(5),
    RED_REEF_LEFT_NEAR(6),
    RED_REEF_CENTER_NEAR(7),
    RED_REEF_RIGHT_NEAR(8),
    RED_REEF_RIGHT_FAR(9),
    RED_REEF_CENTER_FAR(10),
    RED_REEF_LEFT_FAR(11),
    BLUE_CORAL_STATION_RIGHT(12),
    BLUE_CORAL_STATION_LEFT(13),
    BLUE_BARGE_NEAR(14),
    RED_BARGE_FAR(15),
    BLUE_PROCESSOR(16),
    BLUE_REEF_RIGHT_NEAR(17),
    BLUE_REEF_CENTER_NEAR(18),
    BLUE_REEF_LEFT_NEAR(19),
    BLUE_REEF_LEFT_FAR(20),
    BLUE_REEF_CENTER_FAR(21),
    BLUE_REEF_RIGHT_FAR(22);

    private final int id;
    private Translation3d position;

    APRIL_TAG(final int id) {
      this.id = id;
      aprilTagFieldLayout
          .getTagPose(this.id)
          .ifPresent(pose3d -> this.position = pose3d.getTranslation());

      if (this.position == null) {
        throw new IllegalArgumentException("AprilTag ID " + this.id + " does not have a Pose3d!");
      }
    }

    public int getId() {
      return id;
    }

    public Translation3d getTranslation3d() {
      return position;
    }

    public Translation2d getTranslation2d() {
      return position.toTranslation2d();
    }
  }
}
