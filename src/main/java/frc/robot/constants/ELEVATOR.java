package frc.robot.constants;

import edu.wpi.first.math.util.Units;

public class ELEVATOR {

  // TODO: find correct measurements
  // SIM
  // public static final double kDistanceFromIntake = Units.inchesToMeters(17);
  // public static final DCMotor gearbox = DCMotor.getKrakenX60(1);

  public static final double upperLimitMeters = Units.inchesToMeters(21.50);
  public static final double lowerLimitMeters = Units.inchesToMeters(0.0);

  // TODO: Verify gear ratio and sprocket radius
  // public static final double gearRatio = 594.0 / 25.0;
  // public static final double sprocketRadiusMeters = Units.inchesToMeters(1.432) / 2.0;
  // public static final double sprocketRotationsToMeters = sprocketRadiusMeters * 2 * Math.PI;
  // public static final double climberReduction = gearRatio * sprocketRotationsToMeters;
  // public static final double carriageMassKg = 3.0;

  // TODO: figure this out
  // public static final double kMaxVel = Units.inchesToMeters(10);
  // public static final double kMaxAccel = Units.inchesToMeters(18);

  public static final double kP =
      0.1; // none of these are tuned in any capacity so we should probably do that
  public static final double kI = 0.0;
  public static final double kD = 0.0;
  public static final double kA = 0.1;
  public static final double kV = 0.1;
  public static final double kPercentOutputMultiplier = 1.0;
  public static final double kLimitedPercentOutputMultiplier = 0.5;
  public static final double sprocketRadiusMeters = Units.inchesToMeters(1.432) / 2.0;
  public static final double sprocketRotationsToMeters = sprocketRadiusMeters * 2 * Math.PI;

  public enum ELEVATOR_SETPOINT {
    START_POSITION(Units.inchesToMeters(0.0)),
    ALGAE_REEF_INTAKE_LOWER(Units.inchesToMeters(15)),
    ALGAE_REEF_INTAKE_UPPER(Units.inchesToMeters(30)),
    PROCESSOR(Units.inchesToMeters(10)),
    LEVEL_2(Units.inchesToMeters(13)),
    LEVEL_3(Units.inchesToMeters(27)),
    LEVEL_4(Units.inchesToMeters(59.5)),
    NET(Units.inchesToMeters(78));
    private final double setpointMeters;

    ELEVATOR_SETPOINT(double setpointMeters) {
      this.setpointMeters = setpointMeters;
    }

    public double getSetpointMeters() {
      return setpointMeters;
    }
  }
}
