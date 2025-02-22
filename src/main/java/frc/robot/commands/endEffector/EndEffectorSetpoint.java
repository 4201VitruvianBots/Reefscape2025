// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.endEffector;

import java.util.function.Supplier;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.constants.ENDEFFECTOR;
import frc.robot.constants.ROBOT;
import frc.robot.constants.ELEVATOR.ELEVATOR_SETPOINT;
import frc.robot.constants.ENDEFFECTOR.PIVOT.PIVOT_SETPOINT;
import frc.robot.constants.ROBOT.CONTROL_MODE;
import frc.robot.constants.ROBOT.SUPERSTRUCTURE_STATES;
import frc.robot.subsystems.EndEffectorPivot;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class EndEffectorSetpoint extends Command {
  private final EndEffectorPivot m_endEffectorPivot;
  private SUPERSTRUCTURE_STATES m_stage;
  private Supplier<ROBOT.GAME_PIECE> m_selectedGamePiece;

  /** Creates a new EndEffectorSetpoint. */
  public EndEffectorSetpoint(EndEffectorPivot endEffectorPivot, SUPERSTRUCTURE_STATES stage, Supplier<ROBOT.GAME_PIECE> selectedGamePiece) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_endEffectorPivot = endEffectorPivot;
    m_stage = stage;
    m_selectedGamePiece = selectedGamePiece;

    addRequirements(endEffectorPivot);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_endEffectorPivot.setControlMode(CONTROL_MODE.CLOSED_LOOP);
    
    /* If we want to be able to algae toggle in the middle of an outtake cycle, we need to put this in execute().
    No idea why would you want to do that though */
    switch (m_stage) {
      case STOWED:
        m_endEffectorPivot.setPosition(
            PIVOT_SETPOINT.STOWED.get());
        break;
      case L1:
        m_endEffectorPivot.setPosition(
            PIVOT_SETPOINT.STOWED.get());
        break;
      case L2:
        if (m_selectedGamePiece.get() == ROBOT.GAME_PIECE.ALGAE) {
            m_endEffectorPivot.setPosition(PIVOT_SETPOINT.INTAKE_ALGAE_LOW.get());
        } else {
            m_endEffectorPivot.setPosition(
                PIVOT_SETPOINT.L3_L2.get());
        }
        break;
      case L3:
        if (m_selectedGamePiece.get() == ROBOT.GAME_PIECE.ALGAE) {
          m_endEffectorPivot.setPosition(PIVOT_SETPOINT.INTAKE_ALGAE_HIGH.get());
        } else {
          m_endEffectorPivot.setPosition(PIVOT_SETPOINT.L3_L2.get());
        }
        break;
      case L4:
        if (m_selectedGamePiece.get() == ROBOT.GAME_PIECE.ALGAE) {
          m_endEffectorPivot.setPosition(PIVOT_SETPOINT.BARGE.get());
        } else {
          m_endEffectorPivot.setPosition(PIVOT_SETPOINT.L4.get());
        }
        break;
      default:
        break;
    }
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
