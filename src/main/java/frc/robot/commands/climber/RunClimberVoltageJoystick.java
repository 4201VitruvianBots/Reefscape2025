// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.climber;

import java.util.function.DoubleSupplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.constants.ROBOT.CONTROL_MODE;
import frc.robot.subsystems.Climber;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class RunClimberVoltageJoystick extends Command {
  private final Climber m_climber;
  private final DoubleSupplier m_joystickY;

  /** Creates a new RunClimberJoystick. */
  public RunClimberVoltageJoystick(Climber climber, DoubleSupplier joystickY) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_climber = climber;
    m_joystickY = joystickY;
    addRequirements(m_climber);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_climber.setControlMode(CONTROL_MODE.OPEN_LOOP);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double joystickYDeadbandOutput = MathUtil.applyDeadband(m_joystickY.getAsDouble(), 0.1);
    
    m_climber.setJoystickInput(joystickYDeadbandOutput);
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
