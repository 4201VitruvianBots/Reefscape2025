// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.constants;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;

public final class VISION {

  // TODO: figure out if these values need to be changed

  public static final double kLimelightHFOV = 63.12;
  public static final double kLimelightVFOV = 49.54;
  public static final double kLimelightDFOV = 75.07;

  public static final Transform3d[] LOCALIZER_CAMERA_POSITION = {
    // Robot Center to Limelight Camera A
    new Transform3d(
        new Translation3d(
            Units.inchesToMeters(-14), Units.inchesToMeters(6.5), Units.inchesToMeters(14.6875)),
        new Rotation3d()),
    // Robot Center to Limelight Camera B
    new Transform3d(
        new Translation3d(
            Units.inchesToMeters(14), Units.inchesToMeters(-6.5), Units.inchesToMeters(14.6875)),
        new Rotation3d()),
  };

  public static final double aprilTagLimelightCameraADistanceFromCenterX =
      Units.inchesToMeters(-14);
  public static final double aprilTagLimelightCameraADistanceFromCenterY =
      Units.inchesToMeters(6.5);
  public static final double aprilTagLimelightCameraADistanceFromGroundZ =
      Units.inchesToMeters(14.6875);
  public static final double aprilTagLimelightCameraAOffsetInRadiansRoll =
      Units.degreesToRadians(0);
  public static final double aprilTagLimelightCameraAOffsetInRadiansPitch =
      Units.degreesToRadians(0);
  public static final double aprilTagLimelightCameraAOffsetInRadiansYaw = Units.degreesToRadians(0);

  public static final double aprilTagLimelightCameraBDistanceFromCenterX = Units.inchesToMeters(14);
  public static final double aprilTagLimelightCameraBDistanceFromCenterY =
      Units.inchesToMeters(-6.5);
  public static final double aprilTagLimelightCameraBDistanceFromGroundZ =
      Units.inchesToMeters(14.6875);
  public static final double aprilTagLimelightCameraBOffsetInRadiansRoll =
      Units.degreesToRadians(0);
  public static final double aprilTagLimelightCameraBOffsetInRadiansPitch =
      Units.degreesToRadians(0);
  public static final double aprilTagLimelightCameraBOffsetInRadiansYaw = Units.degreesToRadians(0);

  public static final AprilTagFieldLayout aprilTagFieldLayout =
      AprilTagFields.k2025Reefscape.loadAprilTagLayoutField();

  // Camera offset from robot center. Camera A is on the left side of the robot from front view.
  public static final Transform3d robotToAprilTagLimelightCameraA =
      new Transform3d(
          new Translation3d(
              aprilTagLimelightCameraADistanceFromCenterX,
              aprilTagLimelightCameraADistanceFromCenterY,
              aprilTagLimelightCameraADistanceFromGroundZ),
          new Rotation3d(
              aprilTagLimelightCameraAOffsetInRadiansRoll,
              aprilTagLimelightCameraAOffsetInRadiansPitch,
              aprilTagLimelightCameraAOffsetInRadiansYaw));

  // Camera offset from robot center. Camera A is on the left side of the robot from front view.
  public static final Transform3d robotToAprilTagLimelightCameraB =
      new Transform3d(
          new Translation3d(
              aprilTagLimelightCameraBDistanceFromCenterX,
              aprilTagLimelightCameraBDistanceFromCenterY,
              aprilTagLimelightCameraBDistanceFromGroundZ),
          new Rotation3d(
              aprilTagLimelightCameraBOffsetInRadiansRoll,
              aprilTagLimelightCameraBOffsetInRadiansPitch,
              aprilTagLimelightCameraBOffsetInRadiansYaw));

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

  public enum PIPELINE {
    APRILTAGS(0);
    private final int pipeline;

    PIPELINE(final int pipeline) {
      this.pipeline = pipeline;
    }

    public int get() {
      return pipeline;
    }
  }

  public enum CAMERA_SERVER {
    LIMELIGHTA("10.42.1.11"),
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

  // TODO: figure out what value this should be
  public static final double velocityShoot = 9.255586759; // Previously 11.1 m/s
}
