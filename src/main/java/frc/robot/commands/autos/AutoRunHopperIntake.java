package frc.robot.commands.autos;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.constants.HOPPERINTAKE.INTAKE_SPEED;
import frc.robot.subsystems.HopperIntake;

public class AutoRunHopperIntake extends Command {

  private final HopperIntake m_hopperIntake;
  private final INTAKE_SPEED m_speed;

  public AutoRunHopperIntake(HopperIntake hopper, INTAKE_SPEED speed) {
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
  }

  @Override
  public boolean isFinished() {
    return true;
  }
}
