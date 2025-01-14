// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.utils;

import frc.robot.constants.SWERVE;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.Controls;

public class TrajectoryUtils {

  public static  FollowPathCommand generatePPHolonomicCommand(
      CommandSwerveDrivetrain swerveDrive, String pathName, double maxSpeed) {
    PathPlannerPath path = PathPlannerPath.fromPathFile(pathName);

    return generatePPHolonomicCommand(swerveDrive, path, maxSpeed, false);
  }
  

  public static FollowPathHolonomic generatePPHolonomicCommand(
      CommandSwerveDrivetrain swerveDrive, String pathName, double maxSpeed, boolean manualFlip) {
    PathPlannerPath path = PathPlannerPath.fromPathFile(pathName);

    return generatePPHolonomicCommand(swerveDrive, path, maxSpeed, manualFlip);
  }

  public static FollowPathHolonomic generatePPHolonomicCommand(
      CommandSwerveDrivetrain swerveDrive, PathPlannerPath path, double maxSpeed) {

    return generatePPHolonomicCommand(swerveDrive, path, maxSpeed, false);
  }

  public static FollowPathHolonomic generatePPHolonomicCommand(
      CommandSwerveDrivetrain swerveDrive,
      PathPlannerPath path,
      double maxSpeed,
      boolean manualFlip) {
    return new FollowPathHolonomic(
        path,
        () -> swerveDrive.getState().Pose,
        swerveDrive::getChassisSpeed,
        swerveDrive::setChassisSpeedControlNormal,
        new HolonomicPathFollowerConfig(
            new PIDConstants(Swerve.kP_X, TunerConstants.kI_X, TunerConstants.kD_X),
            new PIDConstants(TunerConstants.kAutoP_Theta, TunerConstants.kAutoI_Theta, TunerConstants.kAutoD_Theta),
            maxSpeed,
            //            0.898744,
            SWERVE.DRIVE.kDriveBaseRadius,
            new ReplanningConfig(false, false, 1.0, 0.25)),
        () -> manualFlip || Controls.isRedAlliance(),
        swerveDrive);
  }

  public static FollowPathHolonomic generateStartingPPHolonomicCommand(
      CommandSwerveDrivetrain swerveDrive,
      PathPlannerPath path,
      double maxSpeed,
      boolean manualFlip) {
    return new FollowPathHolonomic(
        path,
        () -> swerveDrive.getState().Pose,
        swerveDrive::getChassisSpeed,
        swerveDrive::setChassisSpeedControlNormal,
        new HolonomicPathFollowerConfig(
            new PIDConstants(DRIVE.kP_X, DRIVE.kI_X, DRIVE.kD_X),
            new PIDConstants(DRIVE.kAutoP_Theta, DRIVE.kAutoI_Theta, DRIVE.kAutoD_Theta),
            maxSpeed,
            //            0.898744,
            SWERVE.DRIVE.kDriveBaseRadius,
            new ReplanningConfig(false, false, 1.0, 0.25)),
        () -> manualFlip || Controls.isRedAlliance(),
        swerveDrive);
  }
}
