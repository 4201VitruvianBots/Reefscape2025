// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.ground;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.constants.GROUND.PIVOT.SETPOINT;
import frc.robot.constants.ROBOT.CONTROL_MODE;
import frc.robot.subsystems.GroundPivot;

public class SetGroundPivotSetpoint extends Command {
  private final GroundPivot m_groundPivot;
  private final SETPOINT m_setpoint;

  /** Creates a new GroundPivotSetpoint. */
  public SetGroundPivotSetpoint(GroundPivot groundPivot, SETPOINT setpoint) {
    m_groundPivot = groundPivot;
    m_setpoint = setpoint;

    addRequirements(m_groundPivot);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_groundPivot.setControlMode(CONTROL_MODE.CLOSED_LOOP);
    m_groundPivot.setDesiredSetpoint(m_setpoint.get());
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
