package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ClimberIntake;

public class RunClimberIntake extends Command {
  private final ClimberIntake m_climberIntake;
  private final double m_desiredPercentOutput;

  public RunClimberIntake(ClimberIntake climberIntake, double desiredPercentOutput) {
    m_climberIntake = climberIntake;
    m_desiredPercentOutput = desiredPercentOutput;
  }

  public void initialize() {
    m_climberIntake.setIntakingState(true);
  }

  public void execute() {
    m_climberIntake.setDesiredPercentOutput(m_desiredPercentOutput);
  }

  public void end(boolean interrupted) {
    m_climberIntake.setDesiredPercentOutput(0);
    m_climberIntake.setIntakingState(false);
  }

  public boolean isFinished() {
    return false;
  }
}
