// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.led;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.constants.LED;
import frc.robot.subsystems.*;

public class GetSubsystemStates extends Command {
  private final LEDSubsystem m_led;
  private final Vision m_vision;
  private final Climber m_climber;
  private final EndEffector
      m_endEffector; // Importing all the subsystems we'll need for this don't worry we'll use this

  // stuff.

  private boolean isEndgame;
  private boolean isLinedUpToReef;
  private boolean isHoldingCoral;
  private boolean isHoldingAlgae;
  private boolean isCoralMode;
  private boolean isAlgaeMode;
  private boolean isEnabled;
  private boolean isDisabled;

  // TODO: Add states. Reference Crescendo code.

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
    // TODO: get/make the states.

    isEndgame = m_climber.getClimbState();
    isLinedUpToReef = false;
    isHoldingCoral = false;
    isHoldingAlgae = false;
    isCoralMode = false;
    isAlgaeMode = false;
    isEnabled = DriverStation.isEnabled();
    isDisabled = DriverStation.isDisabled();

    // the prioritized state to be expressed to the LEDs
    // set in order of priority to be expressed from the least priority to the
    // highest priority
    if (isEndgame) {
      m_led.expressState(LED.SUBSYSTEM_STATES.ENDGAME);
    } else if (isLinedUpToReef) {
      m_led.expressState(LED.SUBSYSTEM_STATES.REEF_LINEUP);
    } else if (isHoldingCoral) {
      m_led.expressState(LED.SUBSYSTEM_STATES.CORAL_OWNED);
    } else if (isHoldingAlgae) {
      m_led.expressState(LED.SUBSYSTEM_STATES.ALGAE_OWNED);
    } else if (isCoralMode) {
      m_led.expressState(LED.SUBSYSTEM_STATES.CORAL);
    } else if (isAlgaeMode) {
      m_led.expressState(LED.SUBSYSTEM_STATES.ALGAE);
    } else if (isEnabled) {
      m_led.expressState(LED.SUBSYSTEM_STATES.ENABLED);
    } else if (isDisabled) {
      m_led.expressState(LED.SUBSYSTEM_STATES.DISABLED);
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
