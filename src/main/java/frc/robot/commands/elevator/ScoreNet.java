package frc.robot.commands.elevator;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.constants.ELEVATOR.ELEVATOR_ACCEL_SETPOINT;
import frc.robot.constants.ROBOT.CONTROL_MODE;
import frc.robot.subsystems.Elevator;

// TODO rework this ENTIRE COMMAND BECAUSE DO WE REALLY NEED THIS AND SETELEVATORSETPOINT
// MYGOSHASGDHGJSADJGSD
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
    m_elevator.setClosedLoopControlMode(CONTROL_MODE.CLOSED_LOOP_NET);
  }

  @Override
  public void execute() {}

  @Override
  public void end(boolean interruped) {
    m_elevator.setClosedLoopControlMode(CONTROL_MODE.OPEN_LOOP);
    m_elevator.holdElevator();
  }

  public boolean isFinished() {
    return false;
  }
}
