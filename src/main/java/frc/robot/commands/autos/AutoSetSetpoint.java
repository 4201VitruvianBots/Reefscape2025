// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.autos;

import static edu.wpi.first.units.Units.Degrees;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.elevator.SetElevatorSetpoint;
import frc.robot.commands.endEffector.EndEffectorSetpoint;
import frc.robot.constants.ELEVATOR.ELEVATOR_SETPOINT;
import frc.robot.constants.ENDEFFECTOR.PIVOT.PIVOT_SETPOINT;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.EndEffectorPivot;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class AutoSetSetpoint extends SequentialCommandGroup {
  /** Creates a new AutoSeSetpoint. */
  private final Elevator m_elevator;

  private final EndEffectorPivot m_endEffectorPivot;

  @SuppressWarnings("static-access")
  public AutoSetSetpoint(Elevator elevator, EndEffectorPivot endEffectorPivot) {
    m_elevator = elevator;
    m_endEffectorPivot = endEffectorPivot;

    ELEVATOR_SETPOINT elevatorSetpoint =
        ELEVATOR_SETPOINT.LEVEL_4; // Replace SOME_SETPOINT with the actual setpoint
    PIVOT_SETPOINT pivotSetpoint =
        PIVOT_SETPOINT.L4; // Replace SOME_SETPOINT with the actual setpoint

    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
        new SequentialCommandGroup(
            new SetElevatorSetpoint(m_elevator, elevatorSetpoint)
                .withTimeout(0.7)
                .onlyWhile(
                    () ->
                        Math.abs(
                                m_elevator.getHeightMeters() - elevatorSetpoint.getSetpointMeters())
                            < Units.inchesToMeters(1)),
            new EndEffectorSetpoint(m_endEffectorPivot, pivotSetpoint)
                .onlyWhile(
                    () ->
                        Math.abs(
                                m_endEffectorPivot.getCurrentRotation().in(Degrees)
                                    - pivotSetpoint.get().in(Degrees))
                            < Units.degreesToRotations(0.5))));
  }
}
