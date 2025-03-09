package frc.robot.constants;

import static edu.wpi.first.units.Units.*;

import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.pathplanner.lib.path.PathConstraints;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.units.measure.Time;
import java.util.Map;
import org.team4201.codex.utils.ModuleMap.MODULE_POSITION;

public class SWERVE {
  public enum MOTOR_TYPE {
    ALL,
    DRIVE,
    STEER
  }

  public static final Distance kWheelBase = Inches.of(23.75);
  public static final Distance kTrackWidth = Inches.of(23.75);
  public static final Distance kBumperThickness = Inches.of(2.5);

  public static final PIDConstants kTranslationPID = new PIDConstants(10, 0, 0);
  public static final PIDConstants kRotationPID = new PIDConstants(7, 0, 0);

  public static final Map<MODULE_POSITION, Translation2d> kModuleTranslations =
      Map.of(
          MODULE_POSITION.FRONT_LEFT,
          new Translation2d(kWheelBase.in(Meters) / 2.0, kTrackWidth.in(Meters) / 2.0),
          MODULE_POSITION.FRONT_RIGHT,
          new Translation2d(kWheelBase.in(Meters) / 2.0, -kTrackWidth.in(Meters) / 2.0),
          MODULE_POSITION.BACK_LEFT,
          new Translation2d(-kWheelBase.in(Meters) / 2.0, kTrackWidth.in(Meters) / 2.0),
          MODULE_POSITION.BACK_RIGHT,
          new Translation2d(-kWheelBase.in(Meters) / 2.0, -kTrackWidth.in(Meters) / 2.0));

  public static final double kMaxSpeedMetersPerSecond = Units.feetToMeters(18);
  public static final double kMaxRotationRadiansPerSecond =
      Math.PI * 0.3; // temporary to reduce speed (original value 2.0) TODO: change back

  // Auto-alignment constants
  public static final PPHolonomicDriveController kDriveController =
      new PPHolonomicDriveController(kTranslationPID, kRotationPID);
  public static final PathConstraints kAutoAlignPathConstraints =
      new PathConstraints(
          1.25, 1.25, 1.0 / 2 * Math.PI, 1 * Math.PI); // The constraints for this path.
  public static final Time kEndTriggerDebounce = Seconds.of(0.04);
  public static final Time kAlignmentAdjustmentTimeout = Seconds.of(0.075);

  public static final Rotation2d kRotationTolerance = Rotation2d.fromDegrees(2.0);
  public static final Distance kPositionTolerance = Inches.of(0.4);
  public static final LinearVelocity kSpeedTolerance = InchesPerSecond.of(0.25);

  public enum ROUTINE_TYPE {
    DRIVE_DYNAMIC(2),
    DRIVE_QUASISTATIC(6),
    TURN_DYNAMIC(8),
    TURN_QUASISTATIC(8);

    private final int lengthSeconds;

    ROUTINE_TYPE(int lengthSeconds) {
      this.lengthSeconds = lengthSeconds;
    }

    public int getLengthSeconds() {
      return lengthSeconds;
    }
  }
}
