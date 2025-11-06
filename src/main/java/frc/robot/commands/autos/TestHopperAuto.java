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
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.commands.RunHopperIntake;
import frc.robot.commands.elevator.SetElevatorSetpoint;
import frc.robot.commands.endEffector.EndEffectorSetpoint;
import frc.robot.commands.endEffector.RunEndEffectorIntake;
import frc.robot.constants.ELEVATOR.ELEVATOR_SETPOINT;
import frc.robot.constants.ENDEFFECTOR.PIVOT.PIVOT_SETPOINT;
import frc.robot.constants.ENDEFFECTOR.ROLLERS.ROLLER_SPEED;
import frc.robot.constants.HOPPERINTAKE;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.EndEffector;
import frc.robot.subsystems.EndEffectorPivot;
import frc.robot.subsystems.HopperIntake;
import org.team4201.codex.simulation.FieldSim;

public class TestHopperAuto extends SequentialCommandGroup {
  /** Creates a new TestHopperAuto. */
  public TestHopperAuto(
      CommandSwerveDrivetrain swerveDrive,
      FieldSim fieldSim,
      Elevator elevator,
      EndEffector endEffector,
      EndEffectorPivot endEffectorPivot,
      HopperIntake hopperIntake) {
    try {
      PathPlannerPath path = PathPlannerPath.fromPathFile("DriveForward");

      var point = new SwerveRequest.PointWheelsAt();

      // Will throw an exception if the starting pose is not prese
      // addCommands(new FooCommand(), new BarCommand());
      addCommands(
          new PrintCommand("path starting point" + path.getStartingHolonomicPose()),
          new PlotAutoPath(swerveDrive, fieldSim, path),
          new ParallelCommandGroup(
                  new RunHopperIntake(hopperIntake, HOPPERINTAKE.INTAKE_SPEED.INTAKING),
                  new RunEndEffectorIntake(endEffector, ROLLER_SPEED.INTAKE_CORAL),
                  new SetElevatorSetpoint(elevator, ELEVATOR_SETPOINT.INTAKE_HOPPER),
                  new EndEffectorSetpoint(endEffectorPivot, PIVOT_SETPOINT.INTAKE_HOPPER))
              .until(endEffector::hasCoral),
          new WaitCommand(0.25),
          new ParallelCommandGroup(
                  new RunEndEffectorIntake(endEffector, ROLLER_SPEED.ZERO),
                  new RunHopperIntake(hopperIntake, HOPPERINTAKE.INTAKE_SPEED.ZERO),
                  new InstantCommand(
                      () ->
                          swerveDrive.applyRequest(
                              () -> point.withModuleDirection(Rotation2d.kZero))))
              .withTimeout(0.1));
    } catch (Exception e) {
      DriverStation.reportError("Failed to load path for DriveForward", e.getStackTrace());
      addCommands(new WaitCommand(0));
    }
  }
}
