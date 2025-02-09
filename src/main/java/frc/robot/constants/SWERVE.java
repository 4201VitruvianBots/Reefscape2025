package frc.robot.constants;

import edu.wpi.first.math.util.Units;
import frc.robot.generated.V2Constants;
import frc.robot.subsystems.CommandSwerveDrivetrain;

public class SWERVE {
  public enum MOTOR_TYPE {
    ALL,
    DRIVE,
    STEER;
  };

  // TODO: Reimplement/add to team library
  //    public static final Map<MODULE_POSITION, Translation2d> kModuleTranslations =
  //            Map.of(
  //                    MODULE_POSITION.FRONT_LEFT,
  //                    new Translation2d(kWheelBase / 2.0, kTrackWidth / 2.0),
  //                    MODULE_POSITION.FRONT_RIGHT,
  //                    new Translation2d(kWheelBase / 2.0, -kTrackWidth / 2.0),
  //                    MODULE_POSITION.BACK_LEFT,
  //                    new Translation2d(-kWheelBase / 2.0, kTrackWidth / 2.0),
  //                    MODULE_POSITION.BACK_RIGHT,
  //                    new Translation2d(-kWheelBase / 2.0, -kTrackWidth / 2.0));
  //
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
