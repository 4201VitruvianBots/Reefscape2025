// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.endEffector;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.EndEffectorPivot;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class EndEffectorBarge extends Command {
  private final EndEffectorPivot m_endEffectorPivot;

  private boolean m_justExitedOpenLoop = false;

  /** Creates a new EndEffectorJoystick. */
  public EndEffectorBarge(EndEffectorPivot endEffectorPivot) {
    m_endEffectorPivot = endEffectorPivot;

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(m_endEffectorPivot);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (m_endEffectorPivot.getAngleDegrees() == 170.0) {
      m_endEffectorPivot.setPercentOutput(0.0);
    } else {
      m_endEffectorPivot.setPercentOutput(0.5);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
