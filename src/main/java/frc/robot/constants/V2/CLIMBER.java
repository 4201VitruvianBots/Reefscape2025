package frc.robot.constants.V2;

import static edu.wpi.first.units.Units.Meters;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Distance;

public class CLIMBER {
  public static final double kP = 0.0;
  public static final double kI = 0.0;
  public static final double kD = 0.0;
  public static final double kPercentOutputMultiplier = 1.0;
  public static final double kLimitedPercentOutputMultiplier = 0.5;
  public static final double sprocketRadiusMeters = Units.inchesToMeters(0.5) / 2.0;
  public static final Distance sprocketRotationsToMeters =
      Meters.of(sprocketRadiusMeters * 2 * Math.PI);
  // public static final double kCarriageMassPounds = 15;
  public static final double gearRatio = 1.0 / 16.0;

  // public static final double kElevatorDrumRadius = Units.inchesToMeters(0.25);
  public static final DCMotor gearbox = DCMotor.getKrakenX60(1);

  public enum CLIMBER_SETPOINT {
    START_POSITION(Units.inchesToMeters(19.356)),
    DEPLOY(Units.inchesToMeters(15.379)),
    INTAKE(Units.inchesToMeters(24.483)),
    CLIMB(Units.inchesToMeters(16.689));
    private final double setpointMeters;

    CLIMBER_SETPOINT(double setpointMeters) {
      this.setpointMeters = setpointMeters;
    }

    public double getSetpointMeters() {
      return setpointMeters;
    }
  }
}
