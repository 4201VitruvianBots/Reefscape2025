// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.autos;

import com.ctre.phoenix6.swerve.SwerveRequest;
import com.pathplanner.lib.path.PathPlannerPath;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import org.team4201.codex.simulation.FieldSim;

public class DriveForward extends SequentialCommandGroup {
  /** Creates a new DriveForward. */
  public DriveForward(CommandSwerveDrivetrain swerveDrive, FieldSim fieldSim) {
    try {
      PathPlannerPath path = PathPlannerPath.fromPathFile("DriveForward");

      var m_ppCommand = swerveDrive.getTrajectoryUtils().generatePPHolonomicCommand("DriveForward");

      var point = new SwerveRequest.PointWheelsAt();
      var stopRequest = new SwerveRequest.ApplyRobotSpeeds();

      // Will throw an exception if the starting pose is not present
      var starting_pose = path.getStartingHolonomicPose().orElseThrow();

      // Add your commands in the addCommands() call, e.g.
      // addCommands(new FooCommand(), new BarCommand());
      addCommands(
          new PrintCommand("path starting point" + path.getStartingHolonomicPose().toString()),
          new PlotAutoPath(swerveDrive, fieldSim, path),
          swerveDrive.getTrajectoryUtils().resetRobotPoseAuto(path),
          new InstantCommand(
                  () -> swerveDrive.applyRequest(() -> point.withModuleDirection(Rotation2d.kZero)),
                  swerveDrive)
              .withTimeout(0.1),
          m_ppCommand.andThen(() -> swerveDrive.setControl(stopRequest)));
    } catch (Exception e) {
      DriverStation.reportError("Failed to load path for DriveForward", e.getStackTrace());
      addCommands(new WaitCommand(0));
    }
  }
}
