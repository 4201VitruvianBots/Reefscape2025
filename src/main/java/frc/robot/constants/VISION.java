// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.constants;

import static edu.wpi.first.units.Units.*;

import com.pathplanner.lib.path.PathConstraints;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.units.measure.Time;

public final class VISION {
  public enum CAMERA_SERVER {
    limelightF("limelight-f", "10.42.1.11"),
    limelightB("limelight-b", "10.42.1.12");

    private final String name;
    private final String ip;

    CAMERA_SERVER(final String name, final String ip) {
      this.name = name;
      this.ip = ip;
    }

    public String getIp() {
      return ip;
    }

    @Override
    public String toString() {
      return name;
    }
  }

  public static final Angle kLimelightHFOV = Degrees.of(63.12);
  public static final Angle kLimelightVFOV = Degrees.of(49.54);
  public static final Angle kLimelightDFOV = Degrees.of(75.07);

  // Camera offset from robot center. Camera F is facing out of the front of the robot (Facing the
  // Hopper)
  public static final Transform3d limelightFPosition =
      new Transform3d(
          new Translation3d(
              Meters.of(0).magnitude(), Meters.of(0).magnitude(), Meters.of(0).magnitude()),
          new Rotation3d(Degrees.of(0), Degrees.of(0), Degrees.of(0)));
  public static final Rotation2d kRotationTolerance = Rotation2d.fromDegrees(2.0);
  public static final Distance kPositionTolerance = Inches.of(0.4);
  public static final LinearVelocity kSpeedTolerance = InchesPerSecond.of(0.25);

  // Camera offset from robot center. Camera B is facing out of the rear of the robot (Facing the
  // EndEffector)
  public static final Transform3d limelightBPosition =
      new Transform3d(
          new Translation3d(
              Meters.of(0).magnitude(), Meters.of(0).magnitude(), Meters.of(0).magnitude()),
          new Rotation3d(Degrees.of(0), Degrees.of(0), Degrees.of(0)));

  public static final Time kEndTriggerDebounce = Seconds.of(0.04);
  public static final Time kAlignmentAdjustmentTimeout = Seconds.of(0.075);
  public static final PathConstraints kPathConstraints =
      new PathConstraints(
          1.25, 1.25, 1.0 / 2 * Math.PI, 1 * Math.PI); // The constraints for this path.

  public static final Distance poseXTolerance = Inches.of(4);
  public static final Distance poseYTolerance = Inches.of(4);
  public static final Distance poseZTolerance = Inches.of(4);
  public static final Angle posePitchTolerance = Degrees.of(4);
  public static final Angle poseRollTolerance = Degrees.of(4);
  public static final Angle poseYawTolerance = Degrees.of(4);

  public enum TRACKING_STATE {
    NONE,
    BRANCH
  }
}
