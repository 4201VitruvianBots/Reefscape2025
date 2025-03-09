// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.constants.ROBOT;
import java.util.function.Consumer;
import java.util.function.Supplier;

public class ToggleGamePiece extends InstantCommand {
  private final Supplier<ROBOT.GAME_PIECE> m_getGamePiece;
  private final Consumer<ROBOT.GAME_PIECE> m_toggleGamePiece;

  /** Creates a new ToggleGamePiece. */
  public ToggleGamePiece(
      Supplier<ROBOT.GAME_PIECE> getGamePiece, Consumer<ROBOT.GAME_PIECE> toggleGamePiece) {
    m_getGamePiece = getGamePiece;
    m_toggleGamePiece = toggleGamePiece;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    if (m_getGamePiece.get() == ROBOT.GAME_PIECE.CORAL) {
      m_toggleGamePiece.accept(ROBOT.GAME_PIECE.ALGAE);
    } else {
      m_toggleGamePiece.accept(ROBOT.GAME_PIECE.CORAL);
    }
  }
}
