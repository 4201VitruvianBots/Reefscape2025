// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.ground;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.constants.ROBOT.CONTROL_MODE;
import frc.robot.subsystems.GroundPivot;

public class SetGroundPivotControlMode extends InstantCommand {
  private final GroundPivot m_groundPivot;
  private CONTROL_MODE m_controlMode;

  /** Set control mode directly */
  public SetGroundPivotControlMode(GroundPivot groundPivot, CONTROL_MODE controlMode) {
    m_groundPivot = groundPivot;
    m_controlMode = controlMode;
  }

  /** Toggle control mode */
  public SetGroundPivotControlMode(GroundPivot groundPivot) {
    this(
        groundPivot,
        groundPivot.getControlMode() == CONTROL_MODE.CLOSED_LOOP
            ? CONTROL_MODE.OPEN_LOOP
            : CONTROL_MODE.CLOSED_LOOP);
  }

  /** Called when the command is initially scheduled. */
  @Override
  public void initialize() {
    m_groundPivot.setControlMode(m_controlMode);
  }

  @Override
  public boolean runsWhenDisabled() {
    return true;
  }
}
