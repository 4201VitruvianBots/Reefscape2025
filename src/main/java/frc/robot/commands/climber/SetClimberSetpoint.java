package frc.robot.commands.climber;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.constants.CLIMBER;
import frc.robot.constants.CLIMBER.CLIMBER_SETPOINT;
import frc.robot.subsystems.Climber;

public class SetClimberSetpoint extends Command {

  private final Climber m_climber;
  private final CLIMBER_SETPOINT m_setpoint;

  public SetClimberSetpoint(Climber climber, CLIMBER_SETPOINT climberSetpoint) {
    m_climber = climber;
    m_setpoint = climberSetpoint;
    addRequirements(m_climber);
  }

  @Override
  public void initialize() {
    m_climber.setDesiredPosition(m_setpoint.getSetpointMeters());
  }

  @Override
  public void execute() {}

  @Override
  public void end(boolean interruped) {}

  public boolean isFinished() {
    return false;
  }
}
