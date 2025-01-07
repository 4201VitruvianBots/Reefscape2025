// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Controls extends SubsystemBase {
  private CommandSwerveDrivetrain m_swervedrive;
  private static DriverStation.Alliance m_allianceColor = DriverStation.Alliance.Red;

  /** Creates a new Controls. */
  public Controls() {}

  public static DriverStation.Alliance getAllianceColor() {
    return m_allianceColor;
  }

  public static boolean isRedAlliance() {
    return (m_allianceColor == DriverStation.Alliance.Red);
  }

  public static boolean isBlueAlliance() {
    return (m_allianceColor == DriverStation.Alliance.Blue);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
