// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.endEffector;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.constants.ENDEFFECTOR.PIVOT.PIVOT_SETPOINT;
import frc.robot.constants.ROBOT.CONTROL_MODE;
import frc.robot.subsystems.EndEffectorPivot;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class EndEffectorSetpoint extends Command {
  private final EndEffectorPivot m_endEffectorPivot;
  private final PIVOT_SETPOINT m_setpoint;

  /** Creates a new EndEffectorSetpoint. */
  public EndEffectorSetpoint(
      EndEffectorPivot endEffectorPivot, PIVOT_SETPOINT setpoint) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_endEffectorPivot = endEffectorPivot;
    m_setpoint = setpoint;

    addRequirements(endEffectorPivot);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_endEffectorPivot.setControlMode(CONTROL_MODE.CLOSED_LOOP);
    m_endEffectorPivot.setPosition(m_setpoint.get());
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
