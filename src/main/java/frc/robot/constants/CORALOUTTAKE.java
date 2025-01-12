package frc.robot.constants;

import edu.wpi.first.math.system.plant.DCMotor;

public class CORALOUTTAKE {
  public static final double kP = 0.1;
  public static final double kI = 0.0;
  public static final double kD = 0.0;
  public static final double gearRatio = 1.0;
  public static final double Inertia = 0.001;
  public static final DCMotor Gearbox = DCMotor.getKrakenX60(1);
}
