// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.endEffector;

import static edu.wpi.first.units.Units.Degrees;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.constants.ENDEFFECTOR;
import frc.robot.constants.ROBOT;
import frc.robot.subsystems.EndEffectorPivot;
import java.util.function.DoubleSupplier;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class EndEffectorJoystick extends Command {
  private final EndEffectorPivot m_endEffectorPivot;
  private final DoubleSupplier m_joystickY;

  /** Creates a new EndEffectorJoystick. */
  public EndEffectorJoystick(EndEffectorPivot endEffectorPivot, DoubleSupplier joystickY) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_endEffectorPivot = endEffectorPivot;
    m_joystickY = joystickY;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // TODO determine if we want to change the deadband and exponent
    double m_joystickDeadband = MathUtil.applyDeadband(Math.pow(m_joystickY.getAsDouble(), 3), 0.2);

    if (m_joystickDeadband != 0.0) {
      if (m_endEffectorPivot.getControlMode() == ROBOT.CONTROL_MODE.CLOSED_LOOP) {
        var rotationSetpoint =
            Degrees.of(
                MathUtil.clamp(
                    m_joystickDeadband * 0.5 + m_endEffectorPivot.getCurrentRotation().in(Degrees),
                    ENDEFFECTOR.minAngle.in(Degrees),
                    ENDEFFECTOR.maxAngle.in(Degrees)));
        m_endEffectorPivot.setPosition(rotationSetpoint);
      } else if (m_endEffectorPivot.getControlMode() == ROBOT.CONTROL_MODE.OPEN_LOOP) {
        if (ENDEFFECTOR.limitOpenLoop) {
          // Upper limit
          if (m_endEffectorPivot.getCurrentRotation().in(Degrees)
              >= ENDEFFECTOR.maxAngle.in(Degrees) - 1)
            m_joystickDeadband = Math.min(m_joystickDeadband, 0);

          // Lower limit
          if (m_endEffectorPivot.getCurrentRotation().in(Degrees)
              <= ENDEFFECTOR.minAngle.in(Degrees) + 1)
            m_joystickDeadband = Math.max(m_joystickDeadband, 0);
        }
        m_endEffectorPivot.setPercentOutput(m_joystickDeadband * ENDEFFECTOR.joystickMultiplier);
      }
    } else {
      if (m_endEffectorPivot.getControlMode() == ROBOT.CONTROL_MODE.OPEN_LOOP)
        m_endEffectorPivot.setPercentOutput(m_joystickDeadband * ENDEFFECTOR.joystickMultiplier);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
