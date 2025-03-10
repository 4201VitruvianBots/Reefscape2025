package frc.robot.commands;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.constants.HOPPERINTAKE.INTAKE_SPEED;
import frc.robot.subsystems.GroundIntake;

public class RunHopperIntake extends Command {

  private final GroundIntake m_hopperIntake;
  private final INTAKE_SPEED m_speed;

  public RunHopperIntake(GroundIntake hopper, INTAKE_SPEED speed) {
    m_hopperIntake = hopper;
    m_speed = speed;
    addRequirements(m_hopperIntake);
  }

  @Override
  public void initialize() {
    m_hopperIntake.setPercentOutput(m_speed.get());
  }

  @Override
  public void execute() {}

  @Override
  public void end(boolean interrupted) {
    if (!DriverStation.isAutonomous()) m_hopperIntake.setPercentOutput(0);
  }

  @Override
  public boolean isFinished() {
    return DriverStation.isAutonomous();
  }
}
