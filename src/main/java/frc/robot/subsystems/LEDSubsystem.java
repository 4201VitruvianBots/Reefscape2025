// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.led.Animation;
import com.ctre.phoenix.led.CANdle;

import edu.wpi.first.wpilibj.util.Color8Bit;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.CAN;

public class LEDSubsystem extends SubsystemBase {
  
   private final CANdle m_candle = new CANdle(CAN.CANdle);

  private Color8Bit m_color = new Color8Bit();
  private int m_red = 0;
  private int m_green = 0; // setting all LED colors to none: there is no color when robot activates
  private int m_blue = 0;
  private int m_white = 0;
  private double m_brightness = 0;
  private double m_speed = 0;
  private SUBSYSTEM_STATES currentRobotState = SUBSYSTEM_STATES.DISABLED;
  private boolean setSolid;
  private Animation m_toAnimate = null;

  public LEDSubsystem() {}

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
