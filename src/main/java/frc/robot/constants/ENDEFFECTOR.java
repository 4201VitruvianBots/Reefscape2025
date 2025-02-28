package frc.robot.constants;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.signals.GravityTypeValue;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.Mass;

public class ENDEFFECTOR {
  public static class ROLLERS {
    public static final double kP = 0.0;
    public static final double kI = 0.0;
    public static final double kD = 0.0;
    public static final double gearRatio = 1.0;
    public static final DCMotor gearbox = DCMotor.getKrakenX60(1);
    public static final double kInertia = 0.001;
    public static final double peakForwardOutput = 0.2;
    public static final double peakReverseOutput = -0.4;

    public enum ROLLER_SPEED {
      // Coral
      OUTTAKE_CORAL(-0.2),
      INTAKE_CORAL(-0.1),
      CORAL_REVERSE(0.2),
      ZERO(0),

      // Algae
      // INTAKE_ALGAE_GROUND(0.35),
      INTAKE_ALGAE_REEF(0.35),
      OUTTAKE_ALGAE_BARGE(0.35),
      OUTTAKE_ALGAE_PROCESSOR(-0.35);

      private final double speed;

      ROLLER_SPEED(final double speed) {
        this.speed = speed;
      }

      public double get() {
        return speed;
      }
    }
  }

  public static class PIVOT {
    // Pivot motor stuff
    public static final double kPivotP = 19.0;
    public static final double kPivotI = 0.0;
    public static final double kPivotD = 0.15;
    public static final double kGPositive = 0.0;
    public static final double kGNegative = 0.0;
    public static final GravityTypeValue K_GRAVITY_TYPE_VALUE = GravityTypeValue.Arm_Cosine;
    public static final double kPivotMotionMagicVelocity = 480;
    public static final double kPivotMotionMagicAcceleration = 90;

    public static final DCMotor pivotGearBox = DCMotor.getKrakenX60(1);
    public static final double pivotGearRatio = 70.0 / 1.0;
    // The values for Distance and Mass are made up
    public static final Distance length = Inches.of(8);
    public static final Mass mass = Pounds.of(5);

    public static final double kPercentOutputMultiplier = 1.0;
    public static final double kLimitedPercentOutputMultiplier = 0.5;

    public static final Angle minAngle = Degrees.of(-10.0);
    public static final Angle maxAngle = Degrees.of(180.0);
    public static final Angle startingAngle = minAngle;
    public static final Angle encoderOffset = Rotations.of(-0.47607421875);

    public static final boolean limitOpenLoop = false;

    public static final double maxOutput = 0.6;

    public static final double joystickMultiplier = maxOutput;

    public static final boolean enforceLimits = true;

    public enum PIVOT_SETPOINT {
      INTAKE_ALGAE_LOW(Degrees.of(170.0)),
      INTAKE_ALGAE_HIGH(Degrees.of(165)),
      INTAKE_HOPPER(Degrees.of(2.0)),
      STOWED(Degrees.of(30.0)),
      L3_L2(Degrees.of(25.0)),
      L4(Degrees.of(60.0)),
      BARGE(Degrees.of(150.0)),
      OUTTAKE_ALGAE_PROCESSOR(
          Degrees.of(
              181.0)); // TODO: Change AbsoluteSensorDiscontinuityPoint to make this angle not
      // negative

      private final Angle angle;

      PIVOT_SETPOINT(final Angle angle) {
        this.angle = angle;
      }

      public Angle get() {
        return angle;
      }
    }
  }

  public enum STATE {
    STILL,
    MOVING
  }
}
