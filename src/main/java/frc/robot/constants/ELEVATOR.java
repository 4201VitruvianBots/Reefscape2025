package frc.robot.constants;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.AngularAcceleration;
import edu.wpi.first.units.measure.AngularVelocity;

import static edu.wpi.first.units.Units.*;

public class ELEVATOR {

  // TODO: find correct measurements
  // SIM
  // public static final double kDistanceFromIntake = Units.inchesToMeters(17);

  public static final double upperLimitMeters = Units.inchesToMeters(78);
  public static final double lowerLimitMeters = Units.inchesToMeters(0);

  // TODO: figure this out
  // public static final double kMaxVel = Units.inchesToMeters(10);
  // public static final double kMaxAccel = Units.inchesToMeters(18);

  public static final double kP =
      15.0; // these are extremely loosely tuned so tune them later
  public static final double kI = 0.0;
  public static final double kD = 0.0;
  public static final double kA = 0.2;
  public static final double kV = 0.1;
  public static final double kG = 500.0;
  public static final double kS = 0.7;
  public static final double offset = 0.05565;
  public static final double motionMagicCruiseVelocity = 100;
  public static final double motionMagicAcceleration = 200;
  public static final double motionMagicJerk = 4000;
  public static final double kPercentOutputMultiplier = 1.0;
  public static final double kLimitedPercentOutputMultiplier = 0.5;
  public static final double sprocketRadiusMeters = Units.inchesToMeters(1.432) / 2.0;
  public static final double sprocketRotationsToMeters = sprocketRadiusMeters * 2 * Math.PI; // Divide the setpoint in meters by this to get rotations. Vice versa to get meters
  public static final double kElevatorGearing = 48.0 / 10;
  public static final double kCarriageMassPounds = 15; // TODO: Change values after CAD done
  public static final double gearRatio = 1.0 / 1.0; // TODO: Change values after CAD done
  public static final double kElevatorDrumRadius = Units.inchesToMeters(1);
  public static final DCMotor gearbox = DCMotor.getKrakenX60(2);

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

  public enum ELEVATOR_ACCEL_SETPOINT {
    NONE(RotationsPerSecondPerSecond.of(0), RotationsPerSecond.of(0)),
    NETSCORE(RotationsPerSecondPerSecond.of(0.5), RotationsPerSecond.of(5)); // Change these later my guy

    private final AngularAcceleration acceleration;
    private final AngularVelocity velocity;
    
    ELEVATOR_ACCEL_SETPOINT(AngularAcceleration acceleration, AngularVelocity velocity) {
      this.acceleration = acceleration;
      this.velocity = velocity;
    }

    public AngularAcceleration getAcceleration() {
      return acceleration;
    }

    public AngularVelocity getVelocity() {
      return velocity;
    }
  }
}
