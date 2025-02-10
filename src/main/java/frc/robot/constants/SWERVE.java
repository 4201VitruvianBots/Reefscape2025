package frc.robot.constants;

import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Meters;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Distance;
import frc.robot.generated.V2Constants;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import java.util.Map;
import org.team4201.codex.utils.ModuleMap.MODULE_POSITION;

public class SWERVE {
  public enum MOTOR_TYPE {
    ALL,
    DRIVE,
    STEER;
  };


  public static Distance kWheelBase = Inches.of(23.75);
  public static Distance kTrackWidth = Inches.of(23.75);
  public static Distance kBumperThickness = Inches.of(2.5);

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

  //    public static final SwerveDriveKinematics kSwerveKinematics =
  //            new SwerveDriveKinematics(
  //                    ModuleMap.orderedValues(kModuleTranslations, new Translation2d[0]));
  public static CommandSwerveDrivetrain selectedDrivetrain;

  public static final double kMaxSpeedMetersPerSecond = Units.feetToMeters(18);
  public static final double kMaxRotationRadiansPerSecond =
      Math.PI * 0.3; // temporary to reduce speed (original value 2.0) TODO: change back

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
