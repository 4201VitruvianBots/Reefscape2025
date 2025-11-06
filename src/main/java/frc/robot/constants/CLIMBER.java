package frc.robot.constants;

import static edu.wpi.first.units.Units.Meters;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Distance;

public class CLIMBER {
  // PID values weren't needed because we weren't using setpoints. 
  public static final double kP = 0.0;
  public static final double kI = 0.0;
  public static final double kD = 0.0;
  public static final double kLimitedPercentOutputMultiplier = 0.5; // Make climber run slower during open loop 
  // See ELEVATOR constants for an explanation of a similar concept of converting distance
  public static final double sprocketRadiusMeters = Units.inchesToMeters(0.5) / 2.0;
  public static final Distance sprocketRotationsToMeters =
      Meters.of(sprocketRadiusMeters * 2.0 * Math.PI);
  public static final double gearRatio = 16.0 / 1.0;

  public static final DCMotor gearbox = DCMotor.getNeo550(1);

  public enum CLIMBER_SETPOINT {
    // Never used because again no setpoints on climber
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
