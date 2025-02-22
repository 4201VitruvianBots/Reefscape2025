package frc.robot.constants;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.AngularAcceleration;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Distance;
import frc.robot.constants.ROBOT.GAME_PIECE;
import frc.robot.constants.ROBOT.SUPERSTRUCTURE_STATES;

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
  public static final double kCarriageMassPounds = 15.0;
  public static final double gearRatio = 48.0 / 10.0;
  public static final DCMotor gearbox = DCMotor.getKrakenX60(2);

  public enum ELEVATOR_SETPOINT {
    START_POSITION(Units.inchesToMeters(0.0), SUPERSTRUCTURE_STATES.STOWED),
    ALGAE_REEF_INTAKE_LOWER(Units.inchesToMeters(21), SUPERSTRUCTURE_STATES.L2, GAME_PIECE.ALGAE),
    ALGAE_REEF_INTAKE_UPPER(Units.inchesToMeters(35), SUPERSTRUCTURE_STATES.L3, GAME_PIECE.ALGAE),
    PROCESSOR(Units.inchesToMeters(10), SUPERSTRUCTURE_STATES.L1, GAME_PIECE.ALGAE),
    LEVEL_2(Units.inchesToMeters(13), SUPERSTRUCTURE_STATES.L2, GAME_PIECE.CORAL),
    LEVEL_3(Units.inchesToMeters(27), SUPERSTRUCTURE_STATES.L3, GAME_PIECE.CORAL),
    LEVEL_4(Units.inchesToMeters(57), SUPERSTRUCTURE_STATES.L4, GAME_PIECE.CORAL),
    NET(Units.inchesToMeters(57.5), SUPERSTRUCTURE_STATES.L4, GAME_PIECE.ALGAE);

    private final double setpointMeters;
    private final SUPERSTRUCTURE_STATES superstructureState;
    private final GAME_PIECE gamePiece;

    ELEVATOR_SETPOINT(
        double setpointMeters, SUPERSTRUCTURE_STATES superstructureState, GAME_PIECE gamePiece) {
      this.setpointMeters = setpointMeters;
      this.superstructureState = superstructureState;
      this.gamePiece = gamePiece;
    }

    ELEVATOR_SETPOINT(double setpointMeters, SUPERSTRUCTURE_STATES superstructureState) {
      this(setpointMeters, superstructureState, GAME_PIECE.NONE);
    }

    public double getSetpointMeters() {
      return setpointMeters;
    }

    public SUPERSTRUCTURE_STATES getSuperstructureState() {
      return superstructureState;
    }

    public GAME_PIECE getGamePiece() {
      return gamePiece;
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
