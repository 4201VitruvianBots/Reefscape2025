/* Copyright (c) FIRST and other WPILib contributors.
Open Source Software; you can modify and/or share it under the terms of
the WPILib BSD license file in the root directory of this project. */

package frc.robot.commands.ground;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Rotations;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.constants.GROUND.PIVOT;
import frc.robot.constants.ROBOT.CONTROL_MODE;
import frc.robot.subsystems.GroundPivot;
import java.util.function.DoubleSupplier;

public class GroundPivotJoystick extends Command {
  /** Creates a new ArmForward. */
  private final GroundPivot m_groundPivot;

  private final DoubleSupplier m_output;

  public GroundPivotJoystick(GroundPivot groundPivot, DoubleSupplier output) {
    m_groundPivot = groundPivot;
    m_output = output;

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(m_groundPivot);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double m_joystickDeadband = MathUtil.applyDeadband(m_output.getAsDouble(), 0.1);
    // double m_joystickDeadband = MathUtil.applyDeadband(Math.pow(m_output.getAsDouble(), 3), 0.2);

    if (m_joystickDeadband != 0.0) {
      if (m_groundPivot.getControlMode() == CONTROL_MODE.CLOSED_LOOP) {
        var rotationSetpoint =
            MathUtil.clamp(
                m_joystickDeadband * 0.5 + m_groundPivot.getCurrentAngle().in(Rotations),
                PIVOT.minAngle.in(Rotations),
                PIVOT.maxAngle.in(Rotations));
        m_groundPivot.setDesiredSetpoint(Rotations.of(rotationSetpoint));
      } else if (m_groundPivot.getControlMode() == CONTROL_MODE.OPEN_LOOP) {
        if (PIVOT.limitOpenLoop) {
          // Upper limit
          if (m_groundPivot.getCurrentAngle().gte(PIVOT.maxAngle.minus(Degrees.of(1))))
            m_joystickDeadband = Math.min(m_joystickDeadband, 0);

          // Lower limit
          if (m_groundPivot.getCurrentAngle().lte(PIVOT.minAngle.plus(Degrees.of(1))))
            m_joystickDeadband = Math.max(m_joystickDeadband, 0);
        }
        m_groundPivot.setPercentOutput(m_joystickDeadband * PIVOT.joystickMultiplier);
      }
    } else {
      if (m_groundPivot.getControlMode() == CONTROL_MODE.OPEN_LOOP)
        m_groundPivot.setPercentOutput(m_joystickDeadband * PIVOT.joystickMultiplier);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_groundPivot.setDesiredSetpoint(m_groundPivot.getCurrentAngle());
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
