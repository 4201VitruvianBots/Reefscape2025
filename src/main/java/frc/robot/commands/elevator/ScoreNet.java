package frc.robot.commands.elevator;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.constants.ELEVATOR.ELEVATOR_ACCEL_SETPOINT;
import frc.robot.constants.ELEVATOR.ELEVATOR_SETPOINT;
import frc.robot.constants.ROBOT.CONTROL_MODE;
import frc.robot.subsystems.Elevator;

public class ScoreNet extends Command {

  private final Elevator m_elevator;
  private final ELEVATOR_ACCEL_SETPOINT m_acceleration;

  public ScoreNet(Elevator elevator, ELEVATOR_ACCEL_SETPOINT acceleration) {
    m_elevator = elevator;
    m_acceleration = acceleration;
    addRequirements(m_elevator);
  }

  @Override
  public void initialize() {
    m_elevator.setControlMode(CONTROL_MODE.NET);
  }

  @Override
  public void execute() {}

  @Override
  public void end(boolean interrupted) {
    m_elevator.setControlMode(CONTROL_MODE.CLOSED_LOOP);
    m_elevator.setDesiredPosition(ELEVATOR_SETPOINT.START_POSITION.getSetpoint());
  }

  public boolean isFinished() {
    return false;
  }
}
