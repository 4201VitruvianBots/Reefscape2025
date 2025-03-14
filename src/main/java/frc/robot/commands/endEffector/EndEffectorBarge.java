// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.endEffector;

import static edu.wpi.first.units.Units.Degrees;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.constants.ROBOT.CONTROL_MODE;
import frc.robot.subsystems.EndEffectorPivot;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class EndEffectorBarge extends Command {
  private final EndEffectorPivot m_endEffectorPivot;

  /** Creates a new EndEffectorJoystick. */
  public EndEffectorBarge(EndEffectorPivot endEffectorPivot) {
    m_endEffectorPivot = endEffectorPivot;

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(m_endEffectorPivot);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_endEffectorPivot.setControlMode(CONTROL_MODE.NET);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (m_endEffectorPivot.getAngleDegrees() >= 170.0) {
      m_endEffectorPivot.setPercentOutput(0.0);
    } else {
      m_endEffectorPivot.setPercentOutput(0.3);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    if (m_endEffectorPivot.getAngle().gt(Degrees.of(170)))
      m_endEffectorPivot.setPosition(Degrees.of(170));
    m_endEffectorPivot.setControlMode(CONTROL_MODE.CLOSED_LOOP);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
