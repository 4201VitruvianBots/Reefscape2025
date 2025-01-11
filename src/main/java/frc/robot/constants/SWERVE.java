package frc.robot.constants;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;

public class SWERVE {
  public static double kTrackWidth = Units.inchesToMeters(21);
  public static final double kWheelBase = Units.inchesToMeters(19);
  public static final double kDriveBaseRadius =
      Math.sqrt(Math.pow(kTrackWidth / 2.0, 2) + Math.pow(kWheelBase / 2.0, 2));

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
  public static final Translation2d kFrontLeftPosition =
      new Translation2d(kWheelBase / 2.0, kTrackWidth / 2.0);
  public static final Translation2d kFrontRightPosition =
      new Translation2d(kWheelBase / 2.0, -kTrackWidth / 2.0);
  public static final Translation2d kBackLeftPosition =
      new Translation2d(-kWheelBase / 2.0, kTrackWidth / 2.0);
  public static final Translation2d kBackRightPosition =
      new Translation2d(-kWheelBase / 2.0, -kTrackWidth / 2.0);

  public static final double kMaxSpeedMetersPerSecond = Units.feetToMeters(18);
  public static final double kMaxRotationRadiansPerSecond = Math.PI * 1.25;

  // TODO: Verify values
  public static final double kDriveMotorGearRatio = 5.36;
  public static final double kTurnMotorGearRatio = 150.0 / 7.0;
  public static final double kCoupleRatio = 3.5714285714285716; // TODO: Verify
  public static final double kWheelRadiusInches = 1.95;
  public static final double kWheelDiameterMeters = 2.0 * Units.inchesToMeters(kWheelRadiusInches);

  public static final boolean[] kTurnInversions = {true, false, false, false};
  public static final boolean[] kDriveInversions = {false, false, false, true};

  // In rotations
  public static double kFrontLeftEncoderOffset = 0.219970703125;
  public static double kFrontRightEncoderOffset = 0.265380859375;
  public static double kBackLeftEncoderOffset = -0.046875;
  public static double kBackRightEncoderOffset = 0.328125;

  public static double kTurnKP = 100;
  public static double kTurnKI = 0;
  public static double kTurnKD = 0.5;
  public static double kTurnKS = 0.1;
  public static double kTurnKV = 1.91;
  public static double kTurnKA = 0;

  public static double kDriveKP = 0.1;
  public static double kDriveKI = 0;
  public static double kDriveKD = 0;
  public static double kDriveKS = 0;
  public static double kDriveKV = 0.124;
  public static double kDriveKA = 0;

  public static final double kTurnSatorCurrentLimit = 60;

  public static final double kSlipCurrent = 120.0;
  public static final double kDriveInertia = 0.01;
  public static final double kTurnInertia = 0.01;
  public static final double kDriveFrictionVoltage = 0.2;
  public static final double kTurnFrictionVoltage = 0.2;
}
