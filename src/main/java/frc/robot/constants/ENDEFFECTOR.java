package frc.robot.constants;

public class ENDEFFECTOR {
  public static final double kP = 0.0;
  public static final double kI = 0.0;
  public static final double kD = 0.0;
  public static final double endEffectorGearRatio = 1.0;

  // Pivot motor stuff
  public static final double minAngleDegrees = 270;
  public static final double maxAngleDegrees = 3;
  
  public static final double kPivotP = 0.0;
  public static final double kPivotI = 0.0;
  public static final double kPivotD = 0.0;

  public static final double endEffectorPivotGearRatio = 70.0;

  public enum WRIST_SETPOINT{
    STOWED(0),
    L4(0),
    L3_L2(0);

    private final double angle;

    WRIST_SETPOINT(double angle){
      this.angle = angle;
    }

    public double get() {
      return angle;
    }
  }

  public enum STATE {
    STILL,
    MOVING
  }
}
