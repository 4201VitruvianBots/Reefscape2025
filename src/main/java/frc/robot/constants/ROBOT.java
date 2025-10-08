package frc.robot.constants;

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
    NET
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
    V3("032398ED"), // 23-2 Rio 2.0

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

    public static ROBOT_ID fromSerial(String serial) {
      for (ROBOT_ID id : ROBOT_ID.values()) {
        if (id.value.equalsIgnoreCase(serial)) return id;
      }
      throw new IllegalArgumentException("Serial " + serial + " not defined in ROBOT_ID!");
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
    PWM.servo = 0;
  }

  public static void initSim() {
    logMode = LOG_MODE.DEBUG;
    robotID = ROBOT_ID.SIM;
  }

  public static void initConstants() {
    var alert = new Alert("Initializing Robot Constants...", AlertType.kInfo);

    try {
      switch (ROBOT_ID.fromSerial(RobotController.getSerialNumber())) {
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
