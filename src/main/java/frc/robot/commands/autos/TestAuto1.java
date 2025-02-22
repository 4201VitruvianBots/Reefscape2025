// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.autos;

import com.ctre.phoenix6.swerve.SwerveRequest;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathPlannerPath;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import org.team4201.codex.simulation.FieldSim;

public class TestAuto1 extends SequentialCommandGroup {
  /**
   * Creates a new DriveForward.
   *
   * @param m_fieldSim
   */
  public TestAuto1(CommandSwerveDrivetrain swerveDrive, FieldSim m_fieldSim) {
    // TODO implement field sim
    try {
      PathPlannerPath path = PathPlannerPath.fromPathFile("4PieceL");

      var m_ppCommand = AutoBuilder.followPath(path);

      var point = new SwerveRequest.PointWheelsAt();
      var stopRequest = new SwerveRequest.ApplyRobotSpeeds();

      // Will throw an exception if the starting pose is not present
      var starting_pose = path.getStartingHolonomicPose().orElseThrow();

      // Add your commands in the addCommands() call, e.g.
      // addCommands(new FooCommand(), new BarCommand());
      addCommands(
          new InstantCommand( // Reset the pose of the robot to the starting pose of the path
              () -> swerveDrive.resetPose(starting_pose)),
          new InstantCommand(
                  () -> swerveDrive.applyRequest(() -> point.withModuleDirection(new Rotation2d())),
                  swerveDrive)
              .alongWith(new WaitCommand(1)),
          m_ppCommand.andThen(() -> swerveDrive.setControl(stopRequest)));
    } catch (Exception e) {
      DriverStation.reportError("Failed to load path for DriveForward", e.getStackTrace());
      addCommands(new WaitCommand(0));
    }
  }
}
