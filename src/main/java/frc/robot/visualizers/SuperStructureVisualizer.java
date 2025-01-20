// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.visualizers;

import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color8Bit;
import frc.robot.constants.ELEVATOR;
import frc.robot.constants.ROBOT;

public class SuperStructureVisualizer {
  private final Mechanism2d m_mech2d = new Mechanism2d(ROBOT.driveBaseLength * 2, ROBOT.driveBaseLength * 2);
  
  private final MechanismRoot2d m_drivebaseRoot2d =
    m_mech2d.getRoot("startRoot", ROBOT.driveBaseLength / 10, ROBOT.driveBaseLength / 10);
  private final MechanismRoot2d m_elevatorRoot2d = 
    m_mech2d.getRoot("elevatorRoot", (ROBOT.driveBaseLength / 10) + ELEVATOR.distanceFromFront, ROBOT.driveBaseLength / 10);
  
  private final MechanismLigament2d m_drivebase2d =
      m_drivebaseRoot2d.append(
          new MechanismLigament2d("drivebase", ROBOT.driveBaseLength, 0));
  
  private final MechanismLigament2d m_elevatorStage1_2d = 
    m_elevatorRoot2d.append(
      new MechanismLigament2d("elevatorStage1", ELEVATOR.stage1Height, 90));
      
  public SuperStructureVisualizer() {
    m_drivebase2d.setColor(new Color8Bit(235, 137, 52));
    m_elevatorStage1_2d.setColor(new Color8Bit(189, 189, 189));
    
    SmartDashboard.putData("SuperStructureSim", m_mech2d);
  }
}
