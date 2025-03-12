// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.epilogue.NotLogged;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import java.util.HashMap;
import java.util.Map;

@Logged
public class Controls extends SubsystemBase {
  private Vision m_vision;
  private static boolean m_allianceInit;
  private static DriverStation.Alliance m_allianceColor = DriverStation.Alliance.Red;

  @NotLogged private final Map<String, Alert> alertMap = new HashMap<>();

  /** Creates a new Controls subsystem */
  public Controls() {
    // Alerts for setting up the robot properly
    alertMap.put(
        "allianceInit",
        new Alert("Did not get alliance color from FMS/DS!", Alert.AlertType.kWarning));
    alertMap.put(
        "visionInit", new Alert("Vision Subsystem is not ready!", Alert.AlertType.kWarning));

    // Alerts for when the robot is running
    alertMap.put("radioError", new Alert("Robot Radio not detected!", Alert.AlertType.kError));
    alertMap.put("joystickError", new Alert("Missing joystick detected!", Alert.AlertType.kError));
    alertMap.put("canError", new Alert("CAN bus error detected!", Alert.AlertType.kError));
    initSmartDashboard();
  }

  public static DriverStation.Alliance getAllianceColor() {
    return m_allianceColor;
  }

  public static boolean isRedAlliance() {
    return getAllianceColor() == DriverStation.Alliance.Red;
  }

  public static boolean isBlueAlliance() {
    return getAllianceColor() == DriverStation.Alliance.Blue;
  }

  private void initSmartDashboard() {
    SmartDashboard.putString("Controls/Serial Number", RobotController.getSerialNumber());
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run

    if (DriverStation.isDisabled()) {
      DriverStation.getAlliance()
          .ifPresent(
              a -> {
                m_allianceColor = a;
                m_allianceInit = true;
              });
      alertMap.get("allianceInit").set(!m_allianceInit);
    }
  }
}
