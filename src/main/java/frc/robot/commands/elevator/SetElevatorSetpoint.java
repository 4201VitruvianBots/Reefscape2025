package frc.robot.commands.elevator;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.constants.ELEVATOR.ELEVATOR_SETPOINT;
import frc.robot.constants.ROBOT;
import frc.robot.constants.ROBOT.CONTROL_MODE;
import frc.robot.subsystems.Elevator;
import java.util.function.Supplier;

public class SetElevatorSetpoint extends Command {

  private final Elevator m_elevator;
  private final ROBOT.SUPERSTRUCTURE_STATES m_stage;
  private final Supplier<ROBOT.GAME_PIECE> m_selectedGamePiece;

  public SetElevatorSetpoint(
      Elevator elevator,
      ROBOT.SUPERSTRUCTURE_STATES stage,
      Supplier<ROBOT.GAME_PIECE> selectedGamePiece) {
    m_elevator = elevator;
    m_stage = stage;
    m_selectedGamePiece = selectedGamePiece;
    addRequirements(m_elevator);
  }

  /* This function is the main bulk of the algae toggle logic for elevator. */
  @Override
  public void initialize() {
    m_elevator.setControlMode(CONTROL_MODE.CLOSED_LOOP);

    /* If we want to be able to algae toggle in the middle of an outtake cycle, we need to put this in execute().
    No idea why would you want to do that though */
    switch (m_stage) {
      case STOWED:
        m_elevator.setDesiredPosition(
            ELEVATOR_SETPOINT.START_POSITION.getSetpointMeters());
        break;
      case L1:
        if (m_selectedGamePiece.get() == ROBOT.GAME_PIECE.ALGAE)
          m_elevator.setDesiredPosition(ELEVATOR_SETPOINT.PROCESSOR.getSetpointMeters());
        // No L1 elevator setpoint right now
        break;
      case L2:
        if (m_selectedGamePiece.get() == ROBOT.GAME_PIECE.ALGAE) {
          m_elevator.setDesiredPosition(
              ELEVATOR_SETPOINT.ALGAE_REEF_INTAKE_LOWER.getSetpointMeters());
        } else {
          m_elevator.setDesiredPosition(ELEVATOR_SETPOINT.LEVEL_2.getSetpointMeters());
        }
        break;
      case L3:
        if (m_selectedGamePiece.get() == ROBOT.GAME_PIECE.ALGAE) {
          m_elevator.setDesiredPosition(
              ELEVATOR_SETPOINT.ALGAE_REEF_INTAKE_UPPER.getSetpointMeters());
        } else {
          m_elevator.setDesiredPosition(ELEVATOR_SETPOINT.LEVEL_3.getSetpointMeters());
        }
        break;
      case L4:
        if (m_selectedGamePiece.get() == ROBOT.GAME_PIECE.ALGAE) {
          m_elevator.setDesiredPosition(ELEVATOR_SETPOINT.NET.getSetpointMeters());
        } else {
          m_elevator.setDesiredPosition(ELEVATOR_SETPOINT.LEVEL_4.getSetpointMeters());
        }
        break;
      default:
        break;
    }
  }

  @Override
  public void execute() {}

  @Override
  public void end(boolean interruped) {}

  @Override
  public boolean isFinished() {
    return false;
  }
}
