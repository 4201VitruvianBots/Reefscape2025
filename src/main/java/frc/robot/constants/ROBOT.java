package frc.robot.constants;

import static edu.wpi.first.units.Units.Pounds;
import static edu.wpi.first.units.Units.Rotations;

import com.ctre.phoenix6.signals.SensorDirectionValue;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj.RobotController;

public class ROBOT {
  public static final boolean useSysID = false;
  // TODO: Change LOG_MODE to Logged.Importance
  public static LOG_MODE logMode = LOG_MODE.NORMAL;
  public static ROBOT_ID robotID = ROBOT_ID.SIM;

  public enum CONTROL_MODE {
    OPEN_LOOP,
    CLOSED_LOOP,
    CLOSED_LOOP_NET
  }

  public enum SUPERSTRUCTURE_STATES {
    STOWED,
    HOPPER_INTAKE,
    L1,
    L2,
    L3,
    L4,
  }

  public enum GAME_PIECE {
    CORAL,
    ALGAE,
    NONE
  }

  public enum ROBOT_ID {
    // Robot Serial Numbers (2023-2024)
    // FORTE - 030cbc95
    // GRIDLOCK - 0306ce62
    // BOBOT - 030e6a97

    // 030cbcf0 - 23-1 Rio 1.0 (in the green bins)
    // 030cbd1c - 23-2 Rio 1.0 (in the green bins)
    // 0310d915 - 23-3 Rio 1.0 (Doesn't work right - Sheraz)

    // Robot Serial Numbers (2025)
    ALPHABOT("030cbc95"), // Rio 1.0
    V2("032381FB"), // 23-1 Rio 2.0
    V3("32398ed"), // 23-2 Rio 2.0

    SIM("");

    private final String value;

    ROBOT_ID(final String value) {
      this.value = value;
    }

    public String getSerial() {
      return value;
    }

    public String getName() {
      return name();
    }

    @Override
    public String toString() {
      return value;
    }
  }

  public static void initAlphaBot() {
    robotID = ROBOT_ID.ALPHABOT;
  }

  public static void initV2() {
    robotID = ROBOT_ID.V2;
  }

  public static void initV3() {
    robotID = ROBOT_ID.V3;

    ELEVATOR.kG = 0.34;
    ELEVATOR.kS = 0.0;
    ELEVATOR.kV = 3.2;
    ELEVATOR.kA = 0.01;
    ELEVATOR.kP = 200;
    ELEVATOR.kI = 0.0;
    ELEVATOR.kD = 0.0;
    ELEVATOR.motionMagicCruiseVelocity = 2000;
    ELEVATOR.motionMagicAcceleration = 4000;
    // ELEVATOR.motionMagicJerk = 2000;
    ELEVATOR.gearbox = DCMotor.getKrakenX60Foc(2);
    ELEVATOR.kCarriageMass = Pounds.of(10.0);

    ENDEFFECTOR.PIVOT.kP = 100.0;
    ENDEFFECTOR.PIVOT.kI = 0.0;
    ENDEFFECTOR.PIVOT.kD = 0.01;
    ENDEFFECTOR.PIVOT.kG = 0.06;
    ENDEFFECTOR.PIVOT.kV = 0.0;
    ENDEFFECTOR.PIVOT.kA = 0.0;
    ENDEFFECTOR.PIVOT.kMotionMagicVelocity = 360;
    ENDEFFECTOR.PIVOT.kMotionMagicAcceleration = 600;
    ENDEFFECTOR.PIVOT.pivotGearBox = DCMotor.getKrakenX60Foc(1);
    ENDEFFECTOR.PIVOT.mass = Pounds.of(7); // TODO: Get actual values
    ENDEFFECTOR.PIVOT.encoderOffset = Rotations.of(0.00341796875);
    ENDEFFECTOR.PIVOT.encoderDirection = SensorDirectionValue.Clockwise_Positive;

    PWM.servo = 0;
  }

  public static void initSim() {
    logMode = LOG_MODE.DEBUG;
    robotID = ROBOT_ID.SIM;
  }

  public static void initConstants() {
    var alert = new Alert("Initializing Robot Constants...", AlertType.kInfo);

    try {
      switch (ROBOT_ID.valueOf(RobotController.getSerialNumber())) {
        case ALPHABOT -> initAlphaBot();
        case V2 -> initV2();
        case V3 -> initV3();
        case SIM -> {
          initSim();
          System.out.print(
              """
                          !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
                          !!! WARNING: This will put logging in debug mode                     !!!
                          !!!          and almost certainly crash the real robot!              !!!
                          !!! IF YOU ARE SEEING THIS IN THE DS CONSOLE, YOUR ROBOT WILL CRASH! !!!
                          !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
                          """);
        }
      }
      alert.setText("Setting Robot Constants for " + robotID.getName());
    } catch (IllegalArgumentException e) {
      alert =
          new Alert(
              "WARN: Robot Serial Not Recognized! Current roboRIO Serial: "
                  + RobotController.getSerialNumber(),
              AlertType.kWarning);
    }
    alert.set(true);
  }

  public enum LOG_MODE {
    DEBUG(0),
    NORMAL(1);

    private final double log_level;

    LOG_MODE(final double log_level) {
      this.log_level = log_level;
    }

    public double get() {
      return log_level;
    }
  }
}
