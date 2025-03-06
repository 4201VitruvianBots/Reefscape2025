package frc.robot.constants;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.AngularAcceleration;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.Mass;

public class ELEVATOR {
  public static final double upperLimitMeters = Units.inchesToMeters(57.5);
  public static final double lowerLimitMeters = Units.inchesToMeters(0);
  public static final Distance superStructureHeight = Inches.of(36.25);

  // TODO: figure this out
  // public static final double kMaxVel = Units.inchesToMeters(10);
  // public static final double kMaxAccel = Units.inchesToMeters(18);

  public static final double kG = 0.36; // output to overcome gravity (output)
  public static final double kS = 0.0; // output to overcome static friction (output)
  public static final double kV = 0.6306; // output per unit of target velocity (output/rps)
  public static final double kA = 0.2; // output per unit of target acceleration (output/(rps/s))
  public static final double kP = 18.5; // output per unit of error in position (output/rotation)
  public static final double kI =
      0.0; // output per unit of integrated error in position (output/rotations*s))
  public static final double kD = 0.1; // output per unit of error in velocity (output/rps)
  public static final double motionMagicCruiseVelocity = 20; // target cruise velocity of 10 rps
  public static final double motionMagicAcceleration = 40; // target acceleration of 20 rps/s
  //   public static final double motionMagicJerk = 4000; // Target jerk of 4000 rps/s/s

  public static final double offset = kG / 12.0; // joystick output to overcome gravity

  public static final double kPercentOutputMultiplier = 1.0; // Upwards motion
  public static final double kLimitedPercentOutputMultiplier = 0.65; // Downwards motion

  public static final double peakForwardOutput = 0.65; // Move up to 1.0 after testing
  public static final double peakReverseOutput = -0.4; // Move up to -0.5-0.65 after testing

  public static final double kElevatorDrumDiameter = Units.inchesToMeters(2.2557);
  public static final double drumRotationsToMeters =
      kElevatorDrumDiameter
          * Math.PI; // Divide the setpoint in meters by this to get rotations. Vice versa to get
  // meters
  public static final Mass kCarriageMass = Pounds.of(15.0);
  public static final double gearRatio = 48.0 / 10.0;
  public static final DCMotor gearbox = DCMotor.getKrakenX60(2);

  public enum ELEVATOR_SETPOINT {
    START_POSITION(Inches.of(0.0)),
    ALGAE_REEF_INTAKE_LOWER(Inches.of(21)),
    ALGAE_REEF_INTAKE_UPPER(Inches.of(35)),
    PROCESSOR(Inches.of(7.25)),
    INTAKE_HOPPER(Inches.of(1.5)), // used to be 3.543
    LEVEL_2(Inches.of(13)),
    LEVEL_3(Inches.of(27)),
    LEVEL_4(Inches.of(57)),
    NET(Inches.of(57.5));

    private final Distance setpoint;

    ELEVATOR_SETPOINT(Distance setpoint) {
      this.setpoint = setpoint;
    }

    public Distance getSetpoint() {
      return setpoint;
    }
  }

  public enum ELEVATOR_ACCEL_SETPOINT {
    NONE(RotationsPerSecondPerSecond.of(0), RotationsPerSecond.of(0)),
    NETSCORE(
        RotationsPerSecondPerSecond.of(0.5),
        RotationsPerSecond.of(5)); // TODO: Change these later my guy

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
