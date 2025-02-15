// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.ground;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.constants.GROUND.PIVOT;
import frc.robot.constants.GROUND.PIVOT.PIVOT_SETPOINT;
import frc.robot.subsystems.GroundPivot;

public class GroundPivotSetpoint extends Command {
  private final GroundPivot m_groundPivot;
  private final PIVOT_SETPOINT m_setpoint;
  private boolean m_auto;

  /** Creates a new GroundPivotSetpoint. */
  public GroundPivotSetpoint(GroundPivot groundPivot, PIVOT_SETPOINT setpoint) {
    m_groundPivot = groundPivot;
    m_setpoint = setpoint;
    m_auto = DriverStation.isAutonomous();

    addRequirements(m_groundPivot);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_groundPivot.setDesiredSetpoint(m_setpoint.get());
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_auto = DriverStation.isAutonomous();
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    if (!m_auto) m_groundPivot.setDesiredSetpoint(PIVOT.PIVOT_SETPOINT.STOWED.get());
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return m_auto;
  }
}
