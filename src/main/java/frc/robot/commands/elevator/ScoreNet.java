package frc.robot.commands.elevator;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.constants.ROBOT.CONTROL_MODE;
import frc.robot.subsystems.Elevator;
import java.util.function.DoubleSupplier;

public class ScoreNet extends Command {

  private final Elevator m_elevator;
  private final DoubleSupplier m_joystickY;

  public ScoreNet(Elevator elevator, DoubleSupplier joystickY) {
    m_elevator = elevator;
    m_joystickY = joystickY;
    addRequirements(m_elevator);
  }

  @Override
  public void initialize() {}

  @Override
  public void execute() {
    
  }

  @Override
  public void end(boolean interruped) {
  }

  public boolean isFinished() {
    return false;
  }
}
