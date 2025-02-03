// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.visualizers;

import static edu.wpi.first.units.Units.*;

import org.team4201.codex.simulation.visualization.Elevator2d;
import org.team4201.codex.simulation.visualization.configs.Elevator2dConfig;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color8Bit;
import frc.robot.constants.ELEVATOR;
import frc.robot.constants.ENDEFFECTOR;
import frc.robot.constants.ROBOT;

public class SuperStructureVisualizer {
  private final Mechanism2d m_mech2d =
      new Mechanism2d(Units.metersToInches(ROBOT.driveBaseLength) * 2, Units.metersToInches(ROBOT.driveBaseLength) * 2);

  private final MechanismRoot2d m_drivebaseRoot2d =
      m_mech2d.getRoot("startRoot", Units.metersToInches(ROBOT.driveBaseLength) / 10, Units.metersToInches(ROBOT.driveBaseLength) / 10);
  private final MechanismRoot2d m_elevatorRoot2d =
      m_mech2d.getRoot(
          "elevatorRoot",
          (Units.metersToInches(ROBOT.driveBaseLength) / 10) + ELEVATOR.distanceFromFront.in(Inches),
          Units.metersToInches(ROBOT.driveBaseLength) / 10);

  private final MechanismLigament2d m_drivebase2d =
      m_drivebaseRoot2d.append(new MechanismLigament2d("drivebase", Units.metersToInches(ROBOT.driveBaseLength), 0));

  private final MechanismLigament2d m_elevatorStage1_2d =
      m_elevatorRoot2d.append(new MechanismLigament2d("elevatorStage1", ELEVATOR.stage1Height.in(Inches), 90));

  // For some reason this elevator shows up as really skinny and way taller than expected
  private final Elevator2d m_elevator2d = new Elevator2d(
    new Elevator2dConfig("elevator", new Color8Bit(0, 180, 0), ELEVATOR.initialCarriageHeight, 2), m_elevatorRoot2d);
  
  private final MechanismLigament2d m_endEffectorBar = m_elevator2d.getLigament(1).append(
    new MechanismLigament2d("endEffectorBar", ENDEFFECTOR.baseBarLength.in(Inches), ENDEFFECTOR.baseBarAngle.in(Degrees)));
  
  /* Super structure parts:
     Static bar extending from the bottom of the elevator to the back, 20 inches at a 54.5 degree angle
     Static bar extending from the back of the drivebase to the other static bar, 15.75 inches at a 75 degree angle (starts 3.6 inches from the middle of the drivebase)

     End effector :D
     Starts 4.2965 inches from the middle of the drivebase
     Extends 8.8 inches from the elevator at a 58 degree angle
  */

  public SuperStructureVisualizer() {
    m_drivebase2d.setColor(new Color8Bit(235, 137, 52));
    m_elevatorStage1_2d.setColor(new Color8Bit(189, 189, 189));
    m_endEffectorBar.setColor(new Color8Bit(255, 0, 0));
    
    m_elevator2d.setAngle(Degrees.of(90));

    SmartDashboard.putData("SuperStructureSim", m_mech2d);
  }
}
