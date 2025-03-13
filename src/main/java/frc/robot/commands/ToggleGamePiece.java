// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.constants.ROBOT.GAME_PIECE;
import frc.robot.subsystems.Controls;

public class ToggleGamePiece extends InstantCommand {
  private final Controls m_controls;

  /** Creates a new ToggleGamePiece. */
  public ToggleGamePiece(Controls controls) {
    m_controls = controls;
  }

  @Override
  public boolean runsWhenDisabled() {
    return true;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    if (m_controls.isGamePieceCoral()) {
      m_controls.setSelectedGamePiece(GAME_PIECE.ALGAE);
    } else {
      m_controls.setSelectedGamePiece(GAME_PIECE.CORAL);
    }
  }
}
