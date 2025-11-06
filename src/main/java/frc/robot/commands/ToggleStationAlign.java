// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.constants.FIELD.TARGET_TYPE;
import frc.robot.subsystems.Vision;

public class ToggleStationAlign extends InstantCommand {
  private final Vision m_vision;

  /** Creates a new ToggleGamePiece. */
  public ToggleStationAlign(Vision vision) {
    m_vision = vision;
  }

  @Override
  public boolean runsWhenDisabled() {
    return true;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    if (m_vision.getTargetType() == TARGET_TYPE.CORAL_STATION) {
      m_vision.setTargetType(TARGET_TYPE.REEF);
    } else {
      m_vision.setTargetType(TARGET_TYPE.CORAL_STATION);
    }
  }
}
