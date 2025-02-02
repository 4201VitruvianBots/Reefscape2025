package frc.robot.constants;

import static edu.wpi.first.units.Units.Degrees;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.measure.Angle;

public class ENDEFFECTOR {
  public static final double kP = 0.0;
  public static final double kI = 0.0;
  public static final double kD = 0.0;
  public static final double endEffectorGearRatio = 1.0;
public static DCMotor endeffectorGearbox = DCMotor.getKrakenX60(1);
public static final double Inertia = 0.001;

  // Pivot motor stuff
  public static final Angle minAngle = Degrees.of(0.0);
  public static final Angle maxAngle = Degrees.of(151.0);
  public static final Angle startingAngle = minAngle;

  public static final double kPivotP = 0.0;
  public static final double kPivotI = 0.0;
  public static final double kPivotD = 0.0;

  public static final double endEffectorPivotGearRatio = 70.0;

  public static final double kPercentOutputMultiplier = 1.0;
  public static final double kLimitedPercentOutputMultiplier = 0.5;

  public enum WRIST_SETPOINT {
    INTAKE_ALGAE(Degrees.of(0.0)),
    STOWED(Degrees.of(30.0)),
    L3_L2(Degrees.of(45.0)),
    L4(Degrees.of(120.0)),
    BARGE(Degrees.of(150.0));

    private final Angle angle;

    WRIST_SETPOINT(final Angle angle) {
      this.angle = angle;
    }

    public Angle get() {
      return angle;
    }
  }

  public enum STATE {
    STILL,
    MOVING
  }
}
