package frc.robot.utils;

import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Meters;

import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color8Bit;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.subsystems.*;
import java.util.HashMap;
import java.util.Map;
import org.team4201.codex.simulation.visualization.Elevator2d;
import org.team4201.codex.simulation.visualization.configs.Elevator2dConfig;

public class Robot2d {
  /**
   * The width/height of the Mechanism2d is scaled based on the window in Glass/SmartDashboard). For
   * consistency, we should just use Inches.
   */
  Mechanism2d m_robot = new Mechanism2d(Inches.of(30).in(Inches), Inches.of(30).in(Inches));

  /** Declare a single point from the main Mechanism2d to attach the chassis */
  MechanismRoot2d m_chassisRoot = m_robot.getRoot("chassisRoot", 0, 0);

  MechanismLigament2d m_robotChassis = new MechanismLigament2d("chassis2d", 0, 0);

  /** Declare a single point from the main Mechanism2d to attach the Elevator2d */
  MechanismRoot2d m_elevatorRoot = m_robot.getRoot("ElevatorRoot", 0, 0);

  /** Create an Elevator2d to represent an elevator, and then attach it to the m_elevatorRoot */
  Elevator2d m_elevator =
      new Elevator2d(
          new Elevator2dConfig("Elevator2d", new Color8Bit(), Inches.of(0)), m_elevatorRoot);

  Map<String, Subsystem> m_subsystemMap = new HashMap<>();

  public Robot2d() {
    // Attach the robotChassis to the chassisRoot
    m_chassisRoot.append(m_robotChassis);

    SmartDashboard.putData("Robot2d", m_robot);

    // For simulation, send the sub-mechanisms to the dashboard.
    // Avoid doing this with the real robot to reduce bandwidth usage.
    // TODO: Just move this logic into Codex
    if (RobotBase.isSimulation()) {
      Mechanism2d m_subElevator2d =
          new Mechanism2d(Inches.of(30).in(Inches), Inches.of(30).in(Inches));
      m_subElevator2d
          .getRoot("subElevator2d", Inches.of(15).in(Inches), Inches.of(0).in(Inches))
          .append(m_elevator.getSubElevator().getLigament());
      SmartDashboard.putData("SubElevator", m_subElevator2d);
    }
  }

  public void registerSubsystem(Subsystem subsystem) {
    m_subsystemMap.put(subsystem.getName(), subsystem);
  }

  public void updateRobot2d() {
    if (m_subsystemMap.containsKey("Elevator")) {
      var elevatorSubsystem = (Elevator) m_subsystemMap.get("Elevator");
      m_elevator.update(Meters.of(elevatorSubsystem.getHeightMeters()));
      // TODO: Add LinearVelocity function in elevator
      //      m_elevator.update(Meters.of(elevatorSubsystem.getHeightMeters()), velocity);
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
