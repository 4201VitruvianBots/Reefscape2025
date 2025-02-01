package frc.robot.commands.alphabot;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.CoralOuttake;

public class RunCoralOuttake extends Command {
  private final CoralOuttake m_coralOuttake;
  private final double m_desiredPercentOutput;

  public RunCoralOuttake(CoralOuttake coralOuttake, double desiredPercentOutput) {
    m_coralOuttake = coralOuttake;
    m_desiredPercentOutput = desiredPercentOutput;
  }

  public void initialize() {
    m_coralOuttake.setOuttakingState(true);
  }

  public void execute() {
    m_coralOuttake.setDesiredPercentOutput(m_desiredPercentOutput);
  }

  public void end(boolean interrupted) {
    m_coralOuttake.setDesiredPercentOutput(0);
    m_coralOuttake.setOuttakingState(false);
  }

  public boolean isFinished() {
    return false;
  }
}
