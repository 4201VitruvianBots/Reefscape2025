// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.swerve;

import com.pathplanner.lib.commands.FollowPathCommand;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.pathplanner.lib.path.GoalEndState;
import com.pathplanner.lib.path.IdealStartingState;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.path.Waypoint;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructPublisher;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.constants.VISION;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.Vision;
import java.util.List;
import java.util.Set;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class DriveToTarget {
  private final CommandSwerveDrivetrain m_swerveDrive;
  private final Vision m_vision;

  private final StructPublisher<Pose2d> desiredBranchPublisher =
      NetworkTableInstance.getDefault()
          .getTable("VisionTracking")
          .getStructTopic("desired branch", Pose2d.struct)
          .publish();

  /** Creates a new DriveToBranch. */
  public DriveToTarget(CommandSwerveDrivetrain swerveDrivetrain, Vision vision) {
    m_swerveDrive = swerveDrivetrain;
    m_vision = vision;
  }

  public Command generateCommand() {
    return Commands.defer(
        () -> {
          // figure out where we need to drive to
          var targetPose = m_vision.getNearestTargetPose();

          // publish the position we want to drive to
          desiredBranchPublisher.accept(targetPose);

          // generate a path to the desired position
          return getPathFromWaypoint(targetPose);
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

    return ppHolonomicCommand(path);
  }

  private Command ppHolonomicCommand(PathPlannerPath path) {
    return new FollowPathCommand(
            path,
            () -> m_swerveDrive.getState().Pose,
            () -> m_swerveDrive.getState().Speeds,
            m_swerveDrive::setChassisSpeedsAuto,
            new PPHolonomicDriveController(
                m_swerveDrive.getAutoTranslationPIDConstants(),
                m_swerveDrive.getAutoRotationPIDConstants()),
            m_swerveDrive.getAutoRobotConfig(),
            () -> false,
            m_swerveDrive)
        .withName("DriveToTarget");
  }
}
