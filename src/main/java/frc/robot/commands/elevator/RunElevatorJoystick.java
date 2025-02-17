package frc.robot.commands.elevator;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.constants.ROBOT.CONTROL_MODE;
import frc.robot.subsystems.Elevator;
import java.util.function.DoubleSupplier;

public class RunElevatorJoystick extends Command {

  private final Elevator m_elevator;
  private final DoubleSupplier m_joystickY;
  
  private boolean m_elevatorHeld = false;

  public RunElevatorJoystick(Elevator elevator, DoubleSupplier joystickY) {
    m_elevator = elevator;
    m_joystickY = joystickY;
    addRequirements(m_elevator);
  }

  @Override
  public void initialize() {}

  @Override
  public void execute() {
    // Adds a Deadband so joystick Ys below 0.1 won't be registered
    double joystickYDeadbandOutput = MathUtil.applyDeadband(m_joystickY.getAsDouble(), 0.1);

    if (joystickYDeadbandOutput != 0.0) {
      m_elevatorHeld = false;
      m_elevator.setControlMode(CONTROL_MODE.OPEN_LOOP);
      m_elevator.setJoystickY(joystickYDeadbandOutput);
    }
    if (joystickYDeadbandOutput == 0) {
      m_elevator.setControlMode(CONTROL_MODE.CLOSED_LOOP);
      if (!m_elevatorHeld) {
        m_elevator.holdElevator();
        m_elevatorHeld = true;
      }
    }
  }

  @Override
  public void end(boolean interruped) {}

  public boolean isFinished() {
    return false;
  }
}
