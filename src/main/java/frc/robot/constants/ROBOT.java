package frc.robot.constants;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj.RobotController;
import frc.robot.Robot;
import frc.robot.generated.AlphaBotConstants;
import frc.robot.generated.V2Constants;

// TODO: this class is a mess with a lot of leftover stuff from Crescendo2024. delete or update
public class ROBOT {
  public static String robotName = "";
  public static final boolean disableVisualization = false;
  public static final boolean useSysID = false;
  public static final boolean useReplayLogs = false;
  public static LOG_MODE logMode = LOG_MODE.NORMAL;
  public static ROBOT_ID robotID = ROBOT_ID.SIM;
  
  public static final double driveBaseWidth = Units.inchesToMeters(29);
  public static final double driveBaseLength = Units.inchesToMeters(29);
  public static final double robotHeight = Units.inchesToMeters(42);

  public enum CONTROL_MODE {
    OPEN_LOOP,
    CLOSED_LOOP
  }

  public enum ROBOT_ID {
    // Robot Serial Numbers (2023-2024)
    // FORTE("030cbc95"),
    // ALPHABOT_OLD("030cbcf0"),
    // GRIDLOCK("0306ce62"),
    // BOBOT("030e6a97"),
    
    // Robot Serial Numbers (2025)
    ALPHABOT("Idk lol"),
    V2("Idk lol"),
    
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

  public enum SETPOINT {
    // Units are in Radians
    STOWED(Units.degreesToRadians(0.0));

    private final double value;

    SETPOINT(final double value) {
      this.value = value;
    }

    public double get() {
      return value;
    }
  }

//   public static void initForte() {}

//   public static void initAlphaBotOld() {}

//   public static void initGridlock() {}

//   public static void initBobot() {}

  public static void initAlphaBot() {
    SWERVE.selectedDrivetrain = AlphaBotConstants.createDrivetrain();
  }
  
  public static void initV2() {
    SWERVE.selectedDrivetrain = V2Constants.createDrivetrain();
  }

  public static void initSim() {
    logMode = LOG_MODE.DEBUG;
    
    // ARM.gearRatio = 1.0; /* Different gear ratios seem to break SimpleJointedArmSim */
  }

  public static void initConstants() {
    var alert = new Alert("Initializing Robot Constants...", AlertType.kInfo);

    // if (RobotController.getSerialNumber().equals(ROBOT_ID.FORTE.getSerial())) {
    //   alert.setText("Setting Robot Constants for FORTE");
    //   initForte();
    // } else if (RobotController.getSerialNumber().equals(ROBOT_ID.ALPHABOT_OLD.getSerial())) {
    //   alert.setText("Setting Robot Constants for ALPHABOT (Old)");
    //   initAlphaBotOld();
    // } else if (RobotController.getSerialNumber().equals(ROBOT_ID.GRIDLOCK.getSerial())) {
    //   alert.setText("Setting Robot Constants for Gridlock");
    //   initGridlock();
    // } else if (RobotController.getSerialNumber().equals(ROBOT_ID.BOBOT.getSerial())) {
    //   alert.setText("Setting Robot Constants for Bobot");
    //   initBobot();
    if (RobotController.getSerialNumber().equals(ROBOT_ID.ALPHABOT.getSerial())) {
      alert.setText("Setting Robot Constants for ALPHABOT");
      initAlphaBot();
    } else if (RobotController.getSerialNumber().equals(ROBOT_ID.ALPHABOT.getSerial())) {
      alert.setText("Setting Robot Constants for V2");
      initV2();
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
