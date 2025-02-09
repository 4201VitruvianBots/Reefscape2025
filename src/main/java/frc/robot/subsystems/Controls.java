// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

@Logged
public class Controls extends SubsystemBase {
  private static DriverStation.Alliance m_allianceColor = DriverStation.Alliance.Red;

  /** Creates a new Controls. */
  public Controls() {
    initSmartDashboard();
  }

  public static DriverStation.Alliance getAllianceColor() {
    return m_allianceColor;
  }

  public static boolean isRedAlliance() {
    return (m_allianceColor == DriverStation.Alliance.Red);
  }

  public static boolean isBlueAlliance() {
    return (m_allianceColor == DriverStation.Alliance.Blue);
  }

  private void initSmartDashboard() {
    SmartDashboard.putString("Controls/Serial Number", RobotController.getSerialNumber());
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run

    if (DriverStation.isDisabled()) {
      var allianceCheck = DriverStation.getAlliance();
      allianceCheck.ifPresent(a -> m_allianceColor = a);
    }
  }
}
