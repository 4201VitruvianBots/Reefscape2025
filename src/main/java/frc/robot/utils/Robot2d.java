package frc.robot.utils;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color8Bit;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.constants.ELEVATOR;
import frc.robot.constants.SWERVE;
import frc.robot.subsystems.*;
import java.util.HashMap;
import java.util.Map;
import org.team4201.codex.simulation.visualization.Elevator2d;
import org.team4201.codex.simulation.visualization.configs.Elevator2dConfig;

/**
 * Class to handle all Mechanism2d updates. The width/height of the Mechanism2d is scaled based on
 * the window in Glass/SmartDashboard). For consistency, we should just use Inches.
 */
public class Robot2d {

  /**
   * Image dimensions 657/830, which ends up being 29 pixels/2 inches. Use this to scale the lineWidth
   * of MechanismLigament2d appropriately
   */
  double pixelsPerInch = 29.0 / 2.0;

  Distance robotCanvasX = Inches.of(45.31034);
  Distance robotCanvasY = Inches.of(57.24138);

  Mechanism2d m_robot = new Mechanism2d(robotCanvasX.magnitude(), robotCanvasY.magnitude());

  /** Declare a single point from the main Mechanism2d to attach the chassis */
  MechanismRoot2d m_chassisRoot =
      m_robot.getRoot("chassisRoot", Inches.of(11).magnitude(), Inches.of(3.75).magnitude());

  /** Use a line (MechanismLigament2d) to represent the robot chassis */
  MechanismLigament2d m_robotChassis =
      new MechanismLigament2d(
          "chassis2d",
          SWERVE.kWheelBase.in(Inches),
          0,
          Inches.of(2).magnitude() * pixelsPerInch,
          new Color8Bit(0, 127, 0));

  /** Declare a single point from the main Mechanism2d to attach the Elevator2d */
  MechanismRoot2d m_elevatorRoot =
      m_robot.getRoot("ElevatorRoot", Inches.of(18.25).magnitude(), Inches.of(3).magnitude());

  /** Create an Elevator2d to represent an elevator, and then attach it to the m_elevatorRoot */
  Elevator2d m_elevator =
      new Elevator2d(
          new Elevator2dConfig("Elevator2d", new Color8Bit(0, 128, 0), Inches.of(0), Degrees.of(90))
              .withLineWidth(Inches.of(2).magnitude() * pixelsPerInch)
                  .withSuperStructureOffset(ELEVATOR.superStructureHeight)
                  .withStageMaxLengths(Meters.of(ELEVATOR.upperLimitMeters))
                  .withStageColors(new Color8Bit(0, 0, 255)),
          m_elevatorRoot);

  /** Map of subsystems for Robot2d to update */
  Map<String, Subsystem> m_subsystemMap = new HashMap<>();

  public Robot2d() {
    // Attach the robotChassis to the chassisRoot
    m_chassisRoot.append(m_robotChassis);

    // Publish Robot2d to SmartDashboard
    SmartDashboard.putData("Robot2d", m_robot);

    // For simulation, create a sub-mechanism display for each mechanism.
    // Avoid doing this with the real robot to reduce bandwidth usage.
    if (RobotBase.isSimulation()) {
      m_elevator.generateSubDisplay();
    }
  }

  public void registerSubsystem(Subsystem subsystem) {
    m_subsystemMap.put(subsystem.getName(), subsystem);
  }

  /** Function to update all mechanisms on Robot2d. This should be called periodically. */
  public void updateRobot2d() {
    if (m_subsystemMap.containsKey("Elevator")) {
      var elevatorSubsystem = (Elevator) m_subsystemMap.get("Elevator");
      m_elevator.update(Meters.of(elevatorSubsystem.getHeightMeters()));
      // TODO: Add LinearVelocity function in elevator
      //   m_elevator.update(Meters.of(elevatorSubsystem.getHeightMeters()), velocity);
    }

    if (m_subsystemMap.containsKey("EndEffectorPivot")) {
      var endEffectorPivotSubsystem = (EndEffectorPivot) m_subsystemMap.get("EndEffectorPivot");
    }

    if (m_subsystemMap.containsKey("EndEffector")) {
      var endEffectorSubsystem = (EndEffector) m_subsystemMap.get("EndEffector");
    }

    if (m_subsystemMap.containsKey("GroundPivot")) {
      var groundPivotSubsystem = (GroundPivot) m_subsystemMap.get("GroundPivot");
    }

    if (m_subsystemMap.containsKey("GroundIntake")) {
      var groundIntakeSubsystem = (GroundIntake) m_subsystemMap.get("GroundIntake");
    }

    if (m_subsystemMap.containsKey("Climber")) {
      var climberSubsystem = (Climber) m_subsystemMap.get("Climber");
    }
  }
}
