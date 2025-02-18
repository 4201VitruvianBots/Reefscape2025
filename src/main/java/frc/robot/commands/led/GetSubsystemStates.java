// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.led;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.constants.LED;
import frc.robot.subsystems.*;

public class GetSubsystemStates extends Command {
  private final LEDSubsystem m_led;
  private final Vision m_vision;
  private final Climber m_climber;
  private final EndEffector m_endEffector; // Importing all the subsystems we'll need for this don't worry we'll use this stuff.

  //TODO: Add states. Reference Crescendo code.
  
  public GetSubsystemStates(
    LEDSubsystem led,
    Vision vision,
    Climber climber,
    EndEffector endEffector) {
      m_led = led;
      m_vision = vision;
      m_climber = climber;
      m_endEffector = endEffector;

      addRequirements(m_led);
  }

  @Override
  public boolean runsWhenDisabled() {
    return true;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    //TODO: Talk with Gavin about the different things.
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
