package frc.robot.commands;

import frc.robot.subsystems.CoralOuttake;

public class runCoralOuttake {
  private final CoralOuttake m_coralOuttake;
  private final double m_desiredPercentOutput;

  public runCoralOuttake(CoralOuttake coralOuttake, double desiredPercentOutput) {
    m_coralOuttake = coralOuttake;
    m_desiredPercentOutput = desiredPercentOutput;
  }

  public void initialize() {
    m_coralOuttake.setOuttakingState(true);
  }

  public void execute() {
    m_coralOuttake.setDesiredPercentOutput(m_desiredPercentOutput);
  }
}
