package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.CageIntake;

public class RunCageIntake extends Command {
  private final CageIntake m_cageIntake;
  private final double m_desiredPercentOutput;

  public RunCageIntake(CageIntake cageIntake, double desiredPercentOutput) {
    m_cageIntake = cageIntake;
    m_desiredPercentOutput = desiredPercentOutput;
  }

  public void initialize() {
    m_cageIntake.setIntakingState(true);
  }

  public void execute() {
    m_cageIntake.setDesiredPercentOutput(m_desiredPercentOutput);
  }

  public void end(boolean interrupted) {
    m_cageIntake.setDesiredPercentOutput(0);
    m_cageIntake.setIntakingState(false);
  }

  public boolean isFinished() {
    return false;
  }
}
