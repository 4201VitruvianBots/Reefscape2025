// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.climber.V2;

import static edu.wpi.first.units.Units.Volts;

import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.constants.ROBOT.CONTROL_MODE;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.V2.V2Climber;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class RunV2ClimberVoltage extends Command {
  private final V2Climber m_v2Climber;
  private final Voltage m_voltage;

  /** Creates a new RunClimber. */
  public RunV2ClimberVoltage(V2Climber v2Climber, Voltage voltage) {
    m_v2Climber = v2Climber;
    m_voltage = voltage;

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(m_v2Climber);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_v2Climber.setControlMode(CONTROL_MODE.OPEN_LOOP);
    m_v2Climber.setButtonInput(m_voltage.in(Volts) / 12);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_v2Climber.setButtonInput(m_voltage.in(Volts) / 12);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_v2Climber.setButtonInput(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
