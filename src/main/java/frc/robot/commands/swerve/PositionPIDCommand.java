// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.swerve;

import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.Seconds;

import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.pathplanner.lib.trajectory.PathPlannerTrajectoryState;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.networktables.BooleanPublisher;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.units.measure.Time;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.constants.SWERVE;
import frc.robot.constants.VISION;
import frc.robot.subsystems.CommandSwerveDrivetrain;

public class PositionPIDCommand extends Command {

  public CommandSwerveDrivetrain m_swerve;
  public final Pose2d goalPose;
  private PPHolonomicDriveController mDriveController = SWERVE.kDriveController;

  private final Trigger endTrigger;
  private final Trigger endTriggerDebounced;

  private final BooleanPublisher endTriggerLogger =
      NetworkTableInstance.getDefault()
          .getTable("logging")
          .getBooleanTopic("PositionPIDEndTrigger")
          .publish();

  private PositionPIDCommand(CommandSwerveDrivetrain m_swerve, Pose2d goalPose) {
    this.m_swerve = m_swerve;
    this.goalPose = goalPose;

    endTrigger =
        new Trigger(
            () -> {
              Pose2d diff = m_swerve.getState().Pose.relativeTo(goalPose);

              var rotation =
                  MathUtil.isNear(
                      0.0,
                      diff.getRotation().getRotations(),
                      VISION.kRotationTolerance.getRotations(),
                      0.0,
                      1.0);

              var position = diff.getTranslation().getNorm() < VISION.kPositionTolerance.in(Meters);

              var speed =
                  m_swerve
                      .getVelocityMagnitude(m_swerve.getChassisSpeed())
                      .lt(VISION.kSpeedTolerance);

              System.out.println(
                  "end trigger conditions R: " + rotation + "\tP: " + position + "\tS: " + speed);

              return rotation && position && speed;
            });

    endTriggerDebounced = endTrigger.debounce(VISION.kEndTriggerDebounce.in(Seconds));
  }

  public static Command generateCommand(
      CommandSwerveDrivetrain m_swerve, Pose2d goalPose, Time timeout) {
    return new PositionPIDCommand(m_swerve, goalPose)
        .withTimeout(timeout)
        .finallyDo(
            () -> {
              m_swerve.setChassisSpeeds(new ChassisSpeeds());
              // m_swerve.lockModules();
            });
  }

  @Override
  public void initialize() {
    endTriggerLogger.accept(endTrigger.getAsBoolean());
  }

  @Override
  public void execute() {
    PathPlannerTrajectoryState goalState = new PathPlannerTrajectoryState();
    goalState.pose = goalPose;

    endTriggerLogger.accept(endTrigger.getAsBoolean());

    m_swerve.setChassisSpeeds(
        mDriveController.calculateRobotRelativeSpeeds(m_swerve.getState().Pose, goalState));
  }

  @Override
  public void end(boolean interrupted) {
    endTriggerLogger.accept(endTrigger.getAsBoolean());
  }

  @Override
  public boolean isFinished() {
    return endTriggerDebounced.getAsBoolean();
  }
}
