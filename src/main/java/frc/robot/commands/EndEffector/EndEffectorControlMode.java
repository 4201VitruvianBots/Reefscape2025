// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.EndEffector;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.constants.ROBOT.CONTROL_MODE;
import frc.robot.subsystems.EndEffectorWrist;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class EndEffectorControlMode extends Command {

  private final EndEffectorWrist m_endEffecotrWrist;
  private CONTROL_MODE m_controlMode;

  /** Creates a new Endefecitr. */
  public EndEffectorControlMode(EndEffectorWrist endEffectorWrist, CONTROL_MODE controlMode) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_endEffecotrWrist = endEffectorWrist;
    m_controlMode = controlMode;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_endEffecotrWrist.setControlMode(m_controlMode);
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
