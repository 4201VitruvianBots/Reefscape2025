// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.autos;

import com.ctre.phoenix6.swerve.SwerveRequest;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.trajectory.PathPlannerTrajectory;
import com.pathplanner.lib.util.PPLibTelemetry;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.Controls;
import org.team4201.codex.simulation.FieldSim;

public class DriveForward extends SequentialCommandGroup {
  /** Creates a new DriveForward. */
  public DriveForward(CommandSwerveDrivetrain swerveDrive, FieldSim fieldSim) {
    try {
      //            var waypoints =
      //                PathPlannerPath.waypointsFromPoses(
      //                    new Pose2d(1, 1, Rotation2d.kZero), new Pose2d(5, 1, Rotation2d.kZero));
      //      var waypoints =
      //          PathPlannerPath.waypointsFromPoses(
      //              new Pose2d(17, 1, Rotation2d.k180deg), new Pose2d(13, 1, Rotation2d.k180deg));
      //      var pathConstraints = new PathConstraints(4, 4, 0.1, 0.1, 12);
      //      var path =
      //          new PathPlannerPath(
      //              waypoints,
      //              pathConstraints,
      //              new IdealStartingState(0, Rotation2d.k180deg),
      //              new GoalEndState(0, Rotation2d.k180deg));
      //      var m_ppCommand =
      //          swerveDrive.getTrajectoryUtils().generatePPHolonomicCommand(path, () -> false);

      var path = PathPlannerPath.fromPathFile("DriveForward");
      var m_ppCommand = swerveDrive.getTrajectoryUtils().generatePPHolonomicCommand(path);

      var point = new SwerveRequest.PointWheelsAt();
      var stopRequest = new SwerveRequest.ApplyRobotSpeeds();

      // Add your commands in the addCommands() call, e.g.
      // addCommands(new FooCommand(), new BarCommand());
      addCommands(
          new PlotAutoPath(swerveDrive, fieldSim, path),
          new InstantCommand(()-> PPLibTelemetry.setCurrentPath(path)),
          new ConditionalCommand(new InstantCommand(
                  () -> swerveDrive.applyRequest(() -> point.withModuleDirection(Rotation2d.k180deg)),
                  swerveDrive),
              new InstantCommand(()->swerveDrive.applyRequest(()->point.withModuleDirection(Rotation2d.kZero))),
                  Controls::isRedAlliance).withTimeout(0.1),
          new WaitCommand(1),
          m_ppCommand.andThen(() -> swerveDrive.setControl(stopRequest)));
    } catch (Exception e) {
      DriverStation.reportError("Failed to load path for DriveForward", e.getStackTrace());
      addCommands(new WaitCommand(0));
    }
  }
}
