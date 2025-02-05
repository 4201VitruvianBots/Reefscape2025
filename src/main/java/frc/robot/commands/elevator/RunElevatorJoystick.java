package frc.robot.commands.elevator;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.constants.ROBOT.CONTROL_MODE;
import frc.robot.subsystems.Elevator;
import java.util.function.DoubleSupplier;

public class RunElevatorJoystick extends Command {

  private final Elevator m_elevator;
  private final DoubleSupplier m_joystickY;

  public RunElevatorJoystick(Elevator elevator, DoubleSupplier joystickY) {
    m_elevator = elevator;
    m_joystickY = joystickY;
    addRequirements(m_elevator);
  }

  @Override
  public void initialize() {}

  @Override
  public void execute() {
    double joystickYDeadbandOutput = MathUtil.applyDeadband(m_joystickY.getAsDouble(), 0.2);

    if (m_elevator.getClosedLoopControlMode() == CONTROL_MODE.OPEN_LOOP) {
      //         if (joystickYDeadbandOutput < 0)
      //           joystickYDeadbandOutput *= CLIMBER.kLimitedPercentOutputMultiplier;
      m_elevator.setJoystickY(joystickYDeadbandOutput);

      if (joystickYDeadbandOutput == 0){
        m_elevator.holdElevator();
      } 
    } else if (m_elevator.getClosedLoopControlMode() == CONTROL_MODE.CLOSED_LOOP) {
      m_elevator.setDesiredPosition(m_elevator.getHeightMeters() + joystickYDeadbandOutput * 0.5);

      if (joystickYDeadbandOutput == 0) m_elevator.holdElevator();
    }
  }

  @Override
  public void end(boolean interruped) {
  }

  public boolean isFinished() {
    return false;
  }
}
