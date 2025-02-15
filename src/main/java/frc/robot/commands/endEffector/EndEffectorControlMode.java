// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.endEffector;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.constants.ROBOT.CONTROL_MODE;
import frc.robot.subsystems.EndEffectorPivot;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class EndEffectorControlMode extends Command {

  private final EndEffectorPivot m_endEffectorPivot;
  private CONTROL_MODE m_controlMode;

  /** Set control mode directly */
  public EndEffectorControlMode(EndEffectorPivot endEffectorPivot, CONTROL_MODE controlMode) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_endEffectorPivot = endEffectorPivot;
    m_controlMode = controlMode;
  }

  /** Toggle control mode */
  public EndEffectorControlMode(EndEffectorPivot endEffectorPivot) {
    this(
        endEffectorPivot,
        endEffectorPivot.getControlMode() == CONTROL_MODE.CLOSED_LOOP
            ? CONTROL_MODE.OPEN_LOOP
            : CONTROL_MODE.CLOSED_LOOP);
  }

  /** Called when the command is initially scheduled. */
  @Override
  public void initialize() {
    m_endEffectorPivot.setControlMode(m_controlMode);
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

  @Override
  public boolean runsWhenDisabled() {
    return true;
  }
}
