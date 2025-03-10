// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.autos;

import com.ctre.phoenix6.swerve.SwerveRequest;
import com.pathplanner.lib.path.PathPlannerPath;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.commands.elevator.SetElevatorSetpoint;
import frc.robot.commands.endEffector.EndEffectorSetpoint;
import frc.robot.commands.endEffector.RunEndEffectorIntake;
import frc.robot.constants.ELEVATOR.ELEVATOR_SETPOINT;
import frc.robot.constants.ENDEFFECTOR.PIVOT.PIVOT_SETPOINT;
import frc.robot.constants.ENDEFFECTOR.ROLLERS.ROLLER_SPEED;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.EndEffector;
import frc.robot.subsystems.EndEffectorPivot;
import frc.robot.subsystems.GroundIntake;
import org.team4201.codex.simulation.FieldSim;

public class OnePieceLeft extends SequentialCommandGroup {
  /** Creates a new TwoPiece. */
  public OnePieceLeft(
      CommandSwerveDrivetrain swerveDrive,
      FieldSim fieldSim,
      Elevator elevator,
      EndEffectorPivot endEffectorPivot,
      EndEffector endEffector,
      GroundIntake hopperIntake) {
    try {
      PathPlannerPath path = PathPlannerPath.fromPathFile("Score1");

      var m_ppCommand = swerveDrive.getTrajectoryUtils().generatePPHolonomicCommand("2PieceLPt1");
      var m_ppCommand2 = swerveDrive.getTrajectoryUtils().generatePPHolonomicCommand("2PieceLback");

      var point = new SwerveRequest.PointWheelsAt();
      var stopRequest = new SwerveRequest.ApplyRobotSpeeds();

      // Will throw an exception if the starting pose is not present
      var starting_pose = path.getStartingHolonomicPose().orElseThrow();

      // Add your commands in the addCommands() call, e.g.
      // addCommands(new FooCommand(), new BarCommand());
      addCommands(
          new PlotAutoPath(swerveDrive, fieldSim, path),
          new ParallelCommandGroup(
                  m_ppCommand.andThen(() -> swerveDrive.setControl(stopRequest)),
                  new ParallelCommandGroup(
                      new SetElevatorSetpoint(elevator, ELEVATOR_SETPOINT.LEVEL_4)
                          .until(elevator::atSetpoint),
                      new EndEffectorSetpoint(endEffectorPivot, PIVOT_SETPOINT.L4)
                          .until(() -> endEffectorPivot.atSetpoint())))
              .withTimeout(2),
          new ParallelCommandGroup(
              new RunEndEffectorIntake(endEffector, ROLLER_SPEED.OUTTAKE_CORAL),
              new WaitCommand(5)),
          new EndEffectorSetpoint(endEffectorPivot, PIVOT_SETPOINT.STOWED).withTimeout(0.7),
          m_ppCommand2.andThen(() -> swerveDrive.setControl(stopRequest)), // move back
          new SetElevatorSetpoint(elevator, ELEVATOR_SETPOINT.START_POSITION)
              .until(elevator::atSetpoint),
          new RunEndEffectorIntake(endEffector, ROLLER_SPEED.ZERO),
          new InstantCommand(
                  () -> swerveDrive.applyRequest(() -> point.withModuleDirection(Rotation2d.kZero)))
              .withTimeout(0.1));
    } catch (Exception e) {
      DriverStation.reportError("Failed to load path for Score1", e.getStackTrace());
      addCommands(new WaitCommand(0));
    }
  }
}
