// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.ground;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.constants.GROUND.DROPDOWN;
import frc.robot.constants.GROUND.DROPDOWN.DROPDOWN_SETPOINT;
import frc.robot.subsystems.GroundDropdown;

public class GroundDropdownSetpoint extends Command {
  private final GroundDropdown m_groundDropdown;
  private final DROPDOWN_SETPOINT m_setpoint;
  private boolean m_auto;

  /** Creates a new GroundDropdownSetpoint. */
  public GroundDropdownSetpoint(GroundDropdown groundDropdown, DROPDOWN_SETPOINT setpoint) {
    m_groundDropdown = groundDropdown;
    m_setpoint = setpoint;
    m_auto = DriverStation.isAutonomous();

    addRequirements(m_groundDropdown);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_groundDropdown.setDesiredSetpoint(m_setpoint.get());
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_auto = DriverStation.isAutonomous();
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    if (!m_auto) m_groundDropdown.setDesiredSetpoint(DROPDOWN.DROPDOWN_SETPOINT.STOWED.get());
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return m_auto;
  }
}
