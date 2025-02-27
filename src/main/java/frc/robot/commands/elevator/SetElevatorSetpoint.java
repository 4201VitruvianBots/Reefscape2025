package frc.robot.commands.elevator;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.constants.ELEVATOR.ELEVATOR_SETPOINT;
import frc.robot.constants.ROBOT.CONTROL_MODE;
import frc.robot.subsystems.Elevator;

public class SetElevatorSetpoint extends Command {

  private final Elevator m_elevator;
  private final ELEVATOR_SETPOINT m_setpoint;

  public SetElevatorSetpoint(Elevator elevator, ELEVATOR_SETPOINT setpoint) {
    m_elevator = elevator;
    m_setpoint = setpoint;
    addRequirements(m_elevator);
  }

  /* This function is the main bulk of the algae toggle logic for elevator. */
  @Override
  public void initialize() {
    m_elevator.setControlMode(CONTROL_MODE.CLOSED_LOOP);
    m_elevator.setDesiredPosition(m_setpoint.getSetpointMeters());
  }

  @Override
  public void execute() {}

  @Override
  public void end(boolean interrupted) {}

  @Override
  public boolean isFinished() {
    return false;
  }
}
