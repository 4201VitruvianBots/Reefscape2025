// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.constants.GROUND.INTAKE.INTAKE_SPEED;
import frc.robot.subsystems.GroundIntake;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class SetGroundIntake extends Command {

  private final GroundIntake m_groundIntake;
  private final INTAKE_SPEED m_speed;

  public SetGroundIntake(GroundIntake ground, INTAKE_SPEED speed) {
    m_groundIntake = ground;
    m_speed = speed;
    addRequirements(m_groundIntake);
  }

  @Override
  public void initialize() {
    m_groundIntake.setPercentOutput(m_speed.get());
  }

  @Override
  public void execute() {}

  @Override
  public void end(boolean interrupted) {
    m_groundIntake.setPercentOutput(0);
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
