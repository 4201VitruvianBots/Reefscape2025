package frc.robot.constants;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.AngleUnit;
import edu.wpi.first.units.DistanceUnit;
import edu.wpi.first.units.measure.*;

public class ELEVATOR {
  public static final Distance upperLimit = Inches.of(57.5);
  public static final Distance lowerLimit = Inches.of(0);
  public static final Distance superStructureHeight = Inches.of(36.25);

  // kG was used rather than gravity type in this case. 
  public static double kG = 0.36; // output to overcome gravity (output)
  public static double kS = 0.0; // output to overcome static friction (output)
  public static double kV = 0.6306; // output per unit of target velocity (output/rps)
  public static double kA = 0.2; // output per unit of target acceleration (output/(rps/s))
  public static double kP = 18.5; // output per unit of error in position (output/rotation)
  public static double kI =
      0.0; // output per unit of integrated error in position (output/rotations*s))
  public static double kD = 0.1; // output per unit of error in velocity (output/rps)
  
  public static double motionMagicCruiseVelocity = 20; // target cruise velocity of 10 rps
  public static double motionMagicAcceleration = 40; // target acceleration of 20 rps/s
  public static double motionMagicJerk = 2000; // Target jerk of 2000 rps/s/s

  public static final double offset = kG / 12.0; // joystick output to overcome gravity

  public static final double kPercentOutputMultiplier = 1.0; // Upwards motion
  public static final double kLimitedPercentOutputMultiplier = 0.65; // Downwards motion

  public static final double peakForwardOutput = 0.65; // Move up to 1.0 after testing
  public static final double peakReverseOutput = -0.4; // Move up to -0.5-0.65 after testing

  // Ask mentors how this is calculated, but basically this is a ratio of how motor motion gets translated into 
  // elevator motion. This can be found by asking hardware usually as well. 
  public static final Distance kElevatorDrumDiameter = Inches.of(2.2557); 
  public static final Distance drumRotationsToDistance =
      kElevatorDrumDiameter.times(
          Math.PI); // Divide the setpoint in meters by this to get rotations. Vice versa to get
  // distance 
  public static final double gearRatio = 48.0 / 10.0;
  public static DCMotor gearbox = DCMotor.getKrakenX60(2);
  public static Mass kCarriageMass = Pounds.of(15.0);

  public static final Per<AngleUnit, DistanceUnit> rotationsToMeters =
      Rotations.of(1).times(gearRatio).div(drumRotationsToDistance);

  public enum ELEVATOR_SETPOINT {
    START_POSITION(Inches.of(0.0)),
    ALGAE_REEF_INTAKE_LOWER(Inches.of(21)),
    ALGAE_REEF_INTAKE_UPPER(Inches.of(35)),
    PROCESSOR(Inches.of(7.25)),
    INTAKE_HOPPER(Inches.of(0.4)), // used to be 3.543
    LEVEL_2(Inches.of(13.0)),
    LEVEL_3(Inches.of(27.0)),
    LEVEL_4(Inches.of(57.0)), // Auto L4 elevator height
    LEVEL_4_TELEOP_SCORE(Inches.of(55.0)), // L4 autoscore and regular L4 in teleop
    NET(Inches.of(57.5));

    private final Distance setpoint;

    ELEVATOR_SETPOINT(Distance setpoint) {
      this.setpoint = setpoint;
    }

    public Distance getSetpoint() {
      return setpoint;
    }
  }
}
