// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.EndEffector;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.constants.ENDEFFECTOR.WRIST_SETPOINT;
import frc.robot.subsystems.EndEffectorWrist;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class EndEffectorSetpoint extends Command {
  private final EndEffectorWrist m_endEffectorWrist;
  private WRIST_SETPOINT m_setpoint;
  private boolean m_auto;
  /** Creates a new EndEffectorSetpoint. */
  public EndEffectorSetpoint(EndEffectorWrist endEffectorWrist, WRIST_SETPOINT setpoint ) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_endEffectorWrist = endEffectorWrist;
    m_setpoint = setpoint;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_endEffectorWrist.setPosition(m_setpoint.get());
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_auto = DriverStation.isAutonomous();
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    if (!m_auto) {
      m_endEffectorWrist.setPosition(WRIST_SETPOINT.STOWED.get());
    }
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
