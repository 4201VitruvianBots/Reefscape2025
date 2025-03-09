// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.constants;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Distance;

public final class VISION {
  public enum CAMERA_SERVER {
    limelightA("limelight-a", "10.42.1.11"),
    limelightB("limelight-a", "10.42.1.12");

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

  // Camera offset from robot center. Camera A is facing out of the front of the robot (Facing the
  // Hopper)
  public static final Transform3d limelightAPosition =
      new Transform3d(
          new Translation3d(
              Meters.of(0).magnitude(), Meters.of(0).magnitude(), Meters.of(0).magnitude()),
          new Rotation3d(Degrees.of(0), Degrees.of(0), Degrees.of(0)));

  // Camera offset from robot center. Camera B is facing out of the rear of the robot (Facing the
  // EndEffector)
  public static final Transform3d limelightBPosition =
      new Transform3d(
          new Translation3d(
              Meters.of(0).magnitude(), Meters.of(0).magnitude(), Meters.of(0).magnitude()),
          new Rotation3d(Degrees.of(0), Degrees.of(0), Degrees.of(0)));

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
