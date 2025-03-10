package frc.robot.constants;

import com.ctre.phoenix6.signals.SensorDirectionValue;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Mass;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj.RobotController;
import frc.robot.Robot;

import static edu.wpi.first.units.Units.Pounds;
import static edu.wpi.first.units.Units.Rotations;

// TODO: this class is a mess with a lot of leftover stuff from Crescendo2024. delete or update
public class ROBOT {
  public static String robotName = "";
  public static final boolean disableVisualization = false;
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
    V2("0382381FB"), // 23-1 Rio 2.0
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
  }

  public static void initAlphaBot() {
    robotID = ROBOT_ID.ALPHABOT;
  }

  public static void initV2() {
    robotID = ROBOT_ID.V2;

    ENDEFFECTOR.PIVOT.mass = Pounds.of(15); // TODO: Get actual values
    ENDEFFECTOR.PIVOT.encoderOffset = Rotations.of(-0.47607421875);
    ENDEFFECTOR.PIVOT.encoderDirection = SensorDirectionValue.CounterClockwise_Positive;
  }

  public static void initV3() {
    robotID = ROBOT_ID.V3;
  }

  public static void initSim() {
    logMode = LOG_MODE.DEBUG;
    robotID = ROBOT_ID.SIM;
  }

  public static void initConstants() {
    var alert = new Alert("Initializing Robot Constants...", AlertType.kInfo);

    if (RobotController.getSerialNumber().equals(ROBOT_ID.ALPHABOT.getSerial())) {
      alert.setText("Setting Robot Constants for ALPHABOT");
      initAlphaBot();
    } else if (RobotController.getSerialNumber().equals(ROBOT_ID.V2.getSerial())) {
      alert.setText("Setting Robot Constants for V2");
      initV2();
    } else if (RobotController.getSerialNumber().equals(ROBOT_ID.V3.getSerial())) {
      alert.setText("Setting Robot Constants for V3");
      initV3();
    } else if (RobotController.getSerialNumber().equals(ROBOT_ID.SIM.getSerial())
        && Robot.isSimulation()) {
      alert.setText("Setting Robot Constants for Sim");
      System.out.println(
          "!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!");
      System.out.println(
          "!!! WARNING: This will put logging in debug mode                     !!!");
      System.out.println(
          "!!!          and almost certainly crash the real robot!              !!!");
      System.out.println(
          "!!! IF YOU ARE SEEING THIS IN THE DS CONSOLE, YOUR ROBOT WILL CRASH! !!!");
      System.out.println(
          "!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!");
      initSim();
    } else {
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
