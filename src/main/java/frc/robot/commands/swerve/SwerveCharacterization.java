// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.swerve;

import com.ctre.phoenix6.swerve.SwerveRequest;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.*;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.constants.SWERVE.ROUTINE_TYPE;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.utils.SysIdUtils;

public class SwerveCharacterization extends SequentialCommandGroup {
  /** Creates a new SwerveCharacterization. */
  public SwerveCharacterization(
      CommandSwerveDrivetrain swerveDrive,
      SysIdRoutine.Direction direction,
      ROUTINE_TYPE routineType) {
    Command sysidCommand;
    SysIdRoutine routine;
    switch (routineType) {
      case DRIVE_DYNAMIC:
        routine = SysIdUtils.getSwerveDriveRoutine();
        sysidCommand = routine.dynamic(direction);
        break;
      case DRIVE_QUASISTATIC:
        routine = SysIdUtils.getSwerveDriveRoutine();
        sysidCommand = routine.quasistatic(direction);
        break;
      case TURN_DYNAMIC:
        routine = SysIdUtils.getSwerveTurnRoutine();
        sysidCommand = routine.dynamic(direction);
        break;
      case TURN_QUASISTATIC:
        routine = SysIdUtils.getSwerveTurnRoutine();
        sysidCommand = routine.quasistatic(direction);
        break;
      default:
        sysidCommand = new InstantCommand();
        throw new IllegalArgumentException("Invalid swerve routine type: " + routineType);
    }

    var point = new SwerveRequest.PointWheelsAt();

    var stopRequest = new SwerveRequest.ApplyRobotSpeeds();

    addCommands(
        new InstantCommand(
            () -> swerveDrive.applyRequest(() -> point.withModuleDirection(new Rotation2d())),
            swerveDrive),
        new WaitCommand(1),
        sysidCommand
            .withTimeout(routineType.getLengthSeconds())
            .andThen(() -> swerveDrive.setControl(stopRequest), swerveDrive));
  }
}
