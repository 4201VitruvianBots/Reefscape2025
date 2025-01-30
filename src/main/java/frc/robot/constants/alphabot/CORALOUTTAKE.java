package frc.robot.constants.alphabot;

import edu.wpi.first.math.system.plant.DCMotor;

public class CORALOUTTAKE {
  public static final double kP = 0.1;
  public static final double kI = 0.0;
  public static final double kD = 0.0;
  public static final double gearRatio = 1.0;
  public static final double inertia = 0.001;
  public static final DCMotor gearbox = DCMotor.getKrakenX60(1);
}
