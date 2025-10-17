// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.climber;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.HopperIntake;

public class RunServo extends Command {
  private final HopperIntake m_hopperintake;
  private final double m_speed;

  /** Creates a new GroundPivotSetpoint. */
  public RunServo(HopperIntake hopperIntake, double speed) {
    m_hopperintake = hopperIntake;
    m_speed = speed;

    addRequirements(m_hopperintake);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_hopperintake.moveServo(m_speed);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_hopperintake.moveServo(0.0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
