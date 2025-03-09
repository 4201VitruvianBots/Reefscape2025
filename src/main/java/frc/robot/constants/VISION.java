// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.constants;

import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.InchesPerSecond;
import static edu.wpi.first.units.Units.Seconds;

import com.pathplanner.lib.path.PathConstraints;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.units.measure.Time;

public final class VISION {

  public static final Rotation2d kRotationTolerance = Rotation2d.fromDegrees(2.0);
  public static final Distance kPositionTolerance = Inches.of(0.4);
  public static final LinearVelocity kSpeedTolerance = InchesPerSecond.of(0.25);

  public static final Time kEndTriggerDebounce = Seconds.of(0.04);
  public static final Time kAlignmentAdjustmentTimeout = Seconds.of(0.075);
  public static final PathConstraints kPathConstraints =
      new PathConstraints(
          1.25, 1.25, 1 / 2 * Math.PI, 1 * Math.PI); // The constraints for this path.

  public enum CAMERA_TYPE {
    LIMELIGHT,
    PHOTONVISION
  }

  public enum CAMERA_SERVER {
    LIMELIGHTF("10.42.1.11"),
    LIMELIGHTB("10.42.1.12"),
    ;
    private final String ip;

    CAMERA_SERVER(final String ip) {
      this.ip = ip;
    }

    @Override
    public String toString() {
      return ip;
    }
  }

  public enum TRACKING_STATE {
    NONE,
    BRANCH
  }
}
