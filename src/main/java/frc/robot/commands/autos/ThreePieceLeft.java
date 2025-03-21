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
import frc.robot.subsystems.HopperIntake;
import org.team4201.codex.simulation.FieldSim;

public class ThreePieceLeft extends SequentialCommandGroup {
  /** Creates a new TwoPiece. */
  public ThreePieceLeft(
      CommandSwerveDrivetrain swerveDrive,
      FieldSim fieldSim,
      Elevator elevator,
      EndEffectorPivot endEffectorPivot,
      EndEffector endEffector,
      HopperIntake hopperIntake) {
    try {
      PathPlannerPath path = PathPlannerPath.fromPathFile("Score1");

      var m_ppCommand = swerveDrive.getTrajectoryUtils().generatePPHolonomicCommand("3PieceLPt1");
      var m_ppCommand2 = swerveDrive.getTrajectoryUtils().generatePPHolonomicCommand("3PieceLback");
      var m_ppCommand3 = swerveDrive.getTrajectoryUtils().generatePPHolonomicCommand("3PieceLPt2");
      var m_ppCommand4 = swerveDrive.getTrajectoryUtils().generatePPHolonomicCommand("3PieceLPt3");
      var m_ppCommand5 = swerveDrive.getTrajectoryUtils().generatePPHolonomicCommand("3PieceLPt4");
      var m_ppCommand6 = swerveDrive.getTrajectoryUtils().generatePPHolonomicCommand("3PieceLPt5");
      var m_ppCommand7 = swerveDrive.getTrajectoryUtils().generatePPHolonomicCommand("3PieceLPt6");
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
                      .until(endEffectorPivot::atSetpoint))),
          new WaitCommand(0.25),
          new ParallelCommandGroup(
              new RunEndEffectorIntake(endEffector, ROLLER_SPEED.AUTOUTTAKE_CORAL),
              new WaitCommand(0.5)),
          new RunEndEffectorIntake(endEffector, ROLLER_SPEED.ZERO),
          m_ppCommand2.andThen(() -> swerveDrive.setControl(stopRequest)),
          // new ParallelCommandGroup(
          //         m_ppCommand3.andThen(() -> swerveDrive.setControl(stopRequest)),
          //         new ParallelCommandGroup(
          //             new RunHopperIntake(hopperIntake, HOPPERINTAKE.INTAKE_SPEED.INTAKING),
          //             new RunEndEffectorIntake(endEffector, ROLLER_SPEED.INTAKE_CORAL),
          //             new SetElevatorSetpoint(elevator, ELEVATOR_SETPOINT.INTAKE_HOPPER),
          //             new EndEffectorSetpoint(endEffectorPivot, PIVOT_SETPOINT.INTAKE_HOPPER)))
          //     .until(endEffector::hasCoral),
          // new WaitCommand(0.25),
          // new ParallelCommandGroup(
          //     new RunEndEffectorIntake(endEffector, ROLLER_SPEED.ZERO),
          //     new RunHopperIntake(hopperIntake, HOPPERINTAKE.INTAKE_SPEED.ZERO)),
          // new ParallelCommandGroup(
          //     m_ppCommand4.andThen(() -> swerveDrive.setControl(stopRequest)),
          //     new ParallelCommandGroup(
          //         new SetElevatorSetpoint(elevator, ELEVATOR_SETPOINT.LEVEL_4)
          //             .until(elevator::atSetpoint),
          //         new EndEffectorSetpoint(endEffectorPivot, PIVOT_SETPOINT.L4)
          //             .until(endEffectorPivot::atSetpoint))),
          // new ParallelCommandGroup(
          //     new RunEndEffectorIntake(endEffector, ROLLER_SPEED.AUTOUTTAKE_CORAL),
          //     new WaitCommand(0.2)),
          //     new ParallelCommandGroup(
          //           m_ppCommand5.andThen(() -> swerveDrive.setControl(stopRequest)),
          //           new ParallelCommandGroup(
          //               new EndEffectorSetpoint(endEffectorPivot, PIVOT_SETPOINT.INTAKE_HOPPER),
          //               new SetElevatorSetpoint(elevator, ELEVATOR_SETPOINT.INTAKE_HOPPER),
          //               new RunHopperIntake(hopperIntake, HOPPERINTAKE.INTAKE_SPEED.INTAKING),
          //               new RunEndEffectorIntake(endEffector, ROLLER_SPEED.INTAKE_CORAL)))
          //       .until(endEffector::hasCoral),
          //   new WaitCommand(0.25),
          //   new ParallelCommandGroup(
          //       new RunEndEffectorIntake(endEffector, ROLLER_SPEED.ZERO),
          //       new RunHopperIntake(hopperIntake, HOPPERINTAKE.INTAKE_SPEED.ZERO)),
          // m_ppCommand6.andThen(() -> swerveDrive.setControl(stopRequest)),
          // m_ppCommand7.andThen(() -> swerveDrive.setControl(stopRequest)),
          new InstantCommand(
                  () -> swerveDrive.applyRequest(() -> point.withModuleDirection(Rotation2d.kZero)))
              .withTimeout(0.1));
    } catch (Exception e) {
      DriverStation.reportError("Failed to load path for Score2", e.getStackTrace());
      addCommands(new WaitCommand(0));
    }
  }
}
