// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.ground;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.constants.ROBOT.CONTROL_MODE;
import frc.robot.subsystems.GroundDropdown;

public class GroundDropdownControlMode extends InstantCommand {
  private final GroundDropdown m_groundDropdown;
  private CONTROL_MODE m_controlMode;

  /** Set control mode directly */
  public GroundDropdownControlMode(GroundDropdown groundDropdown, CONTROL_MODE controlMode) {
    m_groundDropdown = groundDropdown;
    m_controlMode = controlMode;
  }

  /** Toggle control mode */
  public GroundDropdownControlMode(GroundDropdown groundDropdown) {
    this(
        groundDropdown,
        groundDropdown.getControlMode() == CONTROL_MODE.CLOSED_LOOP
            ? CONTROL_MODE.OPEN_LOOP
            : CONTROL_MODE.CLOSED_LOOP);
  }

  /** Called when the command is initially scheduled. */
  @Override
  public void initialize() {
    m_groundDropdown.setControlMode(m_controlMode);
  }

  @Override
  public boolean runsWhenDisabled() {
    return true;
  }
}
