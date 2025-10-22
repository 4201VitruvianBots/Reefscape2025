// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.constants.ROBOT.L4AutoScore;
import frc.robot.subsystems.Controls;

public class ToggleL4Autoscore extends InstantCommand {
  private final Controls m_controls;

  /** Creates a new ToggleGamePiece. */
  public ToggleL4Autoscore(Controls controls) {
    m_controls = controls;
  }

  @Override
  public boolean runsWhenDisabled() {
    return true;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    if (m_controls.isL4AutoScoring() || m_controls.isGamePieceAlgae()){
      m_controls.setL4AutoScore(L4AutoScore.notScoring);
    } else {
      m_controls.setL4AutoScore(L4AutoScore.Scoring);
    }
  }
}
