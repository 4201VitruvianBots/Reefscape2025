// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.swerve;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.GoalEndState;
import com.pathplanner.lib.path.IdealStartingState;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.path.Waypoint;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructPublisher;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.constants.FIELD;
import frc.robot.constants.VISION;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.Controls;
import java.util.Arrays;
import java.util.List;
import java.util.Set;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class DriveToBranch extends Command {

  private final CommandSwerveDrivetrain m_swerveDrive;
  private final StructPublisher<Pose2d> desiredBranchPublisher =
      NetworkTableInstance.getDefault()
          .getTable("VisionTracking")
          .getStructTopic("desired branch", Pose2d.struct)
          .publish();
  private final Pose2d[] robotToNearestBranch = {Pose2d.kZero, Pose2d.kZero};
  private Pose2d nearestBranchPose = Pose2d.kZero;

  /** Creates a new DriveToBranch. */
  public DriveToBranch(CommandSwerveDrivetrain swerveDrivetrain) {
    m_swerveDrive = swerveDrivetrain;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(m_swerveDrive);
  }

  public Command generateCommand() {
    return Commands.defer(
        () -> {
          // figure out where we need to drive to
          robotToNearestBranch[0] = m_swerveDrive.getState().Pose;
          if (Controls.isBlueAlliance()) {
            nearestBranchPose = robotToNearestBranch[0].nearest(Arrays.asList(FIELD.BLUE_BRANCHES));
          } else {
            nearestBranchPose = robotToNearestBranch[0].nearest(Arrays.asList(FIELD.RED_BRANCHES));
          }
          robotToNearestBranch[1] =
              FIELD.REEF_BRANCHES.getBranchPoseToTargetPose(nearestBranchPose);
          // publish the position we want to drive to
          desiredBranchPublisher.accept(robotToNearestBranch[1]);

          // generate a path to the desired position
          return getPathFromWaypoint(getWaypointFromBranch(robotToNearestBranch[1]));
        },
        Set.of());
  }

  private Command getPathFromWaypoint(Pose2d waypoint) {
    List<Waypoint> waypoints =
        PathPlannerPath.waypointsFromPoses(
            new Pose2d(
                m_swerveDrive.getState().Pose.getTranslation(),
                m_swerveDrive.getPathVelocityHeading(m_swerveDrive.getFieldVelocity(), waypoint)),
            waypoint);

    if (waypoints.get(0).anchor().getDistance(waypoints.get(1).anchor()) < 0.01) {
      return Commands.sequence(
          Commands.print("start position PID loop"),
          PositionPIDCommand.generateCommand(
              m_swerveDrive, waypoint, VISION.kAlignmentAdjustmentTimeout),
          Commands.print("end position PID loop"));
    }
    PathPlannerPath path =
        new PathPlannerPath(
            waypoints,
            VISION.kPathConstraints,
            new IdealStartingState(
                m_swerveDrive.getVelocityMagnitude(m_swerveDrive.getChassisSpeed()),
                m_swerveDrive.getState().Pose.getRotation()),
            new GoalEndState(0.0, waypoint.getRotation()));

    path.preventFlipping = true;

    return AutoBuilder.followPath(path)
        .andThen(
            Commands.print("start position PID loop"),
            PositionPIDCommand.generateCommand(
                m_swerveDrive, waypoint, VISION.kAlignmentAdjustmentTimeout),
            Commands.print("end position PID loop"));
  }

  /**
   * @return Pathplanner waypoint with direction of travel away from the associated reef side
   */
  private Pose2d getWaypointFromBranch(Pose2d nearestBranch) {
    return new Pose2d(
        nearestBranch.getTranslation(), nearestBranch.getRotation().rotateBy(Rotation2d.k180deg));
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
