package frc.robot.constants;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.Mass;

public class ENDEFFECTOR {
  public static final double kP = 0.0;
  public static final double kI = 0.0;
  public static final double kD = 0.0;
  public static final double gearRatio = 1.0;
  public static DCMotor gearbox = DCMotor.getKrakenX60(1);
  public static final double kInertia = 0.001;

  // Pivot motor stuff
  public static final double kPivotP = 15.0;
  public static final double kPivotI = 0.0;
  public static final double kPivotD = 0.0;
  public static final double kPivotMotionMagicVelocity = 10;
  public static final double kPivotMotionMagicAcceleration = 10;

  public static final DCMotor pivotGearBox = DCMotor.getKrakenX60(1);
  public static final double pivotGearRatio = 70.0 / 1.0;
  // The values for Distance and Mass are made up
  public static final Distance length = Inches.of(8);
  public static final Mass mass = Pounds.of(5);

  public static final double kPercentOutputMultiplier = 1.0;
  public static final double kLimitedPercentOutputMultiplier = 0.5;

  public static final Angle minAngle = Degrees.of(0.0);
  public static final Angle maxAngle = Degrees.of(151.0);
  public static final Angle startingAngle = minAngle;
  public static final Angle encoderOffset = Rotations.of(-0.47607421875);

  public static final boolean limitOpenLoop = false;

  public static final double maxOutput = 0.6;

  public static final double joystickMultiplier = maxOutput;

  public enum PIVOT_SETPOINT {
    INTAKE_ALGAE(Degrees.of(0.0)),
    STOWED(Degrees.of(30.0)),
    L3_L2(Degrees.of(25.0)),
    L4(Degrees.of(120.0)),
    BARGE(Degrees.of(150.0));

    private final Angle angle;

    PIVOT_SETPOINT(final Angle angle) {
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
