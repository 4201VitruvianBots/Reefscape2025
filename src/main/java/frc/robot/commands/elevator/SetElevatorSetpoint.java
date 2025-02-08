package frc.robot.commands.elevator;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.constants.ELEVATOR;
import frc.robot.constants.ELEVATOR.ELEVATOR_SETPOINT;
import frc.robot.constants.ROBOT.CONTROL_MODE;
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
    m_elevator.setClosedLoopControlMode(CONTROL_MODE.CLOSED_LOOP);
    m_elevator.setDesiredPosition((m_setpoint.getSetpointMeters())/ELEVATOR.sprocketRotationsToMeters);
  }

  @Override
  public void execute() {}

  @Override
  public void end(boolean interruped) {
    m_elevator.holdElevator();
    m_elevator.setClosedLoopControlMode(CONTROL_MODE.OPEN_LOOP);
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
