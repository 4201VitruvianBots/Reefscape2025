// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.Supplier;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.constants.ENDEFFECTOR.ROLLERS.ROLLER_SPEED;
import frc.robot.constants.ROBOT;
import frc.robot.subsystems.EndEffector;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class RunEndEffectorIntake extends Command {

  private final EndEffector m_endEffector;
  private final boolean m_intaking;
  private Supplier<ROBOT.GAME_PIECE> m_selectedGamePiece;

  /** Creates a new RunEndEffectorIntake. */
  public RunEndEffectorIntake(EndEffector endEffector, boolean intaking, Supplier<ROBOT.GAME_PIECE> selectedGamePiece) {
    m_endEffector = endEffector;
    m_intaking = intaking;
    m_selectedGamePiece = selectedGamePiece;
    
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(m_endEffector);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    switch (m_selectedGamePiece.get()) {
        case CORAL:
            if (m_intaking) {
                m_endEffector.setPercentOutput(ROLLER_SPEED.INTAKE_CORAL_HOPPER.get());
            } else {
                m_endEffector.setPercentOutput(ROLLER_SPEED.CORAL_REEF_REVERSE.get());
            }
        case ALGAE:
            if (m_intaking) {
                m_endEffector.setPercentOutput(ROLLER_SPEED.INTAKE_ALGAE_REEF.get());
            } else {
                m_endEffector.setPercentOutput(ROLLER_SPEED.OUTTAKE_ALGAE_PROCESSOR.get());
            }
        default:
        break;
    }
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_endEffector.setPercentOutput(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
