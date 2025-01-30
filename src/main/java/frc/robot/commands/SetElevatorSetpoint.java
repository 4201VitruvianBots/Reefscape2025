package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.constants.ELEVATOR.ELEVATOR_SETPOINT;
import frc.robot.subsystems.Elevator;

public class SetElevatorSetpoint extends Command {

  private final Elevator m_elevator;
  private final ELEVATOR_SETPOINT m_setpoint;

  public SetElevatorSetpoint(Elevator elevator, ELEVATOR_SETPOINT elevatorSetpoint) {
    m_elevator = elevator;
    m_setpoint = elevatorSetpoint;
    addRequirements(m_elevator);
  }

  @Override
  public void initialize() {
    m_elevator.setDesiredPosition(m_setpoint.getSetpointMeters());
  }

  @Override
  public void execute() {}

  @Override
  public void end(boolean interruped) {}

  public boolean isFinished() {
    return false;
  }
}
