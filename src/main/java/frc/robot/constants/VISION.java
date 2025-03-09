// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.constants;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Meters;

import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;

public final class VISION {
  public static final double kLimelightHFOV = 63.12;
  public static final double kLimelightVFOV = 49.54;
  public static final double kLimelightDFOV = 75.07;

  // Camera offset from robot center. Camera A is on the left side of the robot from front view.
  public static final Transform3d limelightAPosition =
      new Transform3d(
          new Translation3d(
              Meters.of(0).magnitude(), Meters.of(0).magnitude(), Meters.of(0).magnitude()),
          new Rotation3d(Degrees.of(0), Degrees.of(0), Degrees.of(0)));

  // Camera offset from robot center. Camera A is on the left side of the robot from front view.
  public static final Transform3d limelightBPosition =
      new Transform3d(
          new Translation3d(
              Meters.of(0).magnitude(), Meters.of(0).magnitude(), Meters.of(0).magnitude()),
          new Rotation3d(Degrees.of(0), Degrees.of(0), Degrees.of(0)));

  public static final double poseXTolerance = Units.inchesToMeters(4);
  public static final double poseYTolerance = Units.inchesToMeters(4);
  public static final double poseZTolerance = Units.inchesToMeters(4);
  public static final double posePitchTolerance = Units.degreesToRadians(4);
  public static final double poseRollTolerance = Units.degreesToRadians(4);
  public static final double poseYawTolerance = Units.degreesToRadians(4);

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
