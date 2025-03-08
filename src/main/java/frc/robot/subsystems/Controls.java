// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.USB;

@Logged
public class Controls extends SubsystemBase {
  private static DriverStation.Alliance m_allianceColor = DriverStation.Alliance.Red;
  
  private static Alert m_usbAlert = new Alert("The following USB devices are not connected: ", AlertType.kError);

  /** Creates a new Controls. */
  public Controls() {
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

  private void updateAlerts() {
    // Update USB alerts
    m_usbAlert.set(false);
    
    // String for USB alert message
    String usbAlertMessage = "The following USB devices are not connected: ";
    
    if (!DriverStation.isJoystickConnected(USB.leftJoystick)) {
        usbAlertMessage += "Left Joystick, ";
        m_usbAlert.set(true);
    }
    if (!DriverStation.isJoystickConnected(USB.rightJoystick)) {
        usbAlertMessage += "Right Joystick, ";
        m_usbAlert.set(true);
    }
    if (!DriverStation.isJoystickConnected(USB.xBoxController)) {
        usbAlertMessage += "Xbox Controller, ";
        m_usbAlert.set(true);
    }
    
    // Remove the last comma and space
    if (usbAlertMessage.endsWith(", ")) {
        usbAlertMessage = usbAlertMessage.substring(0, usbAlertMessage.length() - 2);
    }
    m_usbAlert.setText(usbAlertMessage);
  }
  
  @Override
  public void periodic() {
    // This method will be called once per scheduler run

    if (DriverStation.isDisabled()) {
      DriverStation.getAlliance().ifPresent(a -> m_allianceColor = a);
    }
    
    updateAlerts();
  }
}
