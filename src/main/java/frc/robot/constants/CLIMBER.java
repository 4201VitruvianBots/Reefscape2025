package frc.robot.constants;

import edu.wpi.first.math.system.plant.DCMotor;

public class CLIMBER {
  public static final double kP = 0.0;
  public static final double kI = 0.0;
  public static final double kD = 0.0;
  public static final double climberGearRatio = 1.0;
  public static final double sprocketRotationsToMeters = 0;
  public static final double kPercentOutputMultiplier = 0;

  public class INTAKE {
    public static final double kP = 0.0;
    public static final double kI = 0.0;
    public static final double kD = 0.0;
    public static final double gearRatio = 1 / 32;
    public static final DCMotor gearbox = DCMotor.getKrakenX60(1);
  }
}
