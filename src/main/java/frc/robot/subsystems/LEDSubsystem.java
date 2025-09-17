// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.led.Animation;
import com.ctre.phoenix.led.CANdle;
import com.ctre.phoenix.led.CANdle.LEDStripType;
import com.ctre.phoenix.led.CANdle.VBatOutputMode;
import com.ctre.phoenix.led.CANdleConfiguration;
import com.ctre.phoenix.led.CANdleStatusFrame;
import com.ctre.phoenix.led.ColorFlowAnimation;
import com.ctre.phoenix.led.ColorFlowAnimation.Direction;
import com.ctre.phoenix.led.FireAnimation;
import com.ctre.phoenix.led.LarsonAnimation;
import com.ctre.phoenix.led.LarsonAnimation.BounceMode;
import com.ctre.phoenix.led.RainbowAnimation;
import com.ctre.phoenix.led.RgbFadeAnimation;
import com.ctre.phoenix.led.SingleFadeAnimation;
import com.ctre.phoenix.led.StrobeAnimation;
import com.ctre.phoenix.led.TwinkleAnimation;
import com.ctre.phoenix.led.TwinkleAnimation.TwinklePercent;
import com.ctre.phoenix.led.TwinkleOffAnimation;
import com.ctre.phoenix.led.TwinkleOffAnimation.TwinkleOffPercent;
import edu.wpi.first.wpilibj.util.Color8Bit;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.CAN;
import frc.robot.constants.LED;
import frc.robot.constants.LED.*;

public class LEDSubsystem extends SubsystemBase {
  /** Creates a new LEDSubsystem. */
  private final CANdle m_candle = new CANdle(CAN.CANdle, CAN.driveBaseCanbus);

  private Color8Bit m_color = new Color8Bit();
  private int m_red = 0;
  private int m_green = 0; // setting all LED colors to none: there is no color when robot activates
  private int m_blue = 0;
  private int m_white = 0; // we have a separate bulb in our LED strips for white
  private double m_brightness = 0;
  private double m_speed = 0;
  private SUBSYSTEM_STATES currentRobotState = SUBSYSTEM_STATES.DISABLED;
  private boolean setSolid; // this stops the animation
  private Animation m_toAnimate = null;

  public LEDSubsystem() {
    m_candle.configFactoryDefault();
    // sets up LED strip
    CANdleConfiguration configAll = new CANdleConfiguration();
    configAll.statusLedOffWhenActive = true; // sets lights of when the LEDs are activated
    configAll.disableWhenLOS = false; // disables LEDs when there is no signal for control
    configAll.stripType = LEDStripType.GRB;
    configAll.brightnessScalar =
        0.5; // 1 is highest we can go we don't want to blind everyone at the event
    configAll.vBatOutputMode = VBatOutputMode.Modulated; // Modulate
    m_candle.configAllSettings(configAll, 100);
    m_candle.setStatusFramePeriod(CANdleStatusFrame.CANdleStatusFrame_Status_1_General, 255);
    m_candle.setStatusFramePeriod(CANdleStatusFrame.CANdleStatusFrame_Status_2_Startup, 255);
    m_candle.setStatusFramePeriod(
        CANdleStatusFrame.CANdleStatusFrame_Status_3_FirmwareApiStatus, 255);
    m_candle.setStatusFramePeriod(CANdleStatusFrame.CANdleStatusFrame_Status_4_ControlTelem, 255);
    m_candle.setStatusFramePeriod(
        CANdleStatusFrame.CANdleStatusFrame_Status_5_PixelPulseTrain, 255);
    m_candle.setStatusFramePeriod(CANdleStatusFrame.CANdleStatusFrame_Status_6_BottomPixels, 255);
    m_candle.setStatusFramePeriod(CANdleStatusFrame.CANdleStatusFrame_Status_7_TopPixels, 255);
  }

  // will create LED patterns
  public void setPattern(Color8Bit color, int white, double speed, ANIMATION_TYPE toChange) {
    m_color = color;
    m_red = color.red;
    m_green = color.green;
    m_blue = color.blue;
    m_white = white;
    m_speed = speed;
    switch (toChange) {
      case ColorFlow: // stripe of color flowing through the LED strip
        m_toAnimate =
            new ColorFlowAnimation(
                m_red, m_green, m_blue, m_white, m_speed, LED.LEDcount, Direction.Forward);
        break;
      case ColorFlowLong: // stripe of color flowing through the LED strip
        m_toAnimate =
            new ColorFlowAnimation(
                m_red, m_green, m_blue, m_white, m_speed, LED.LEDcount + 15, Direction.Forward);
        break;

      case Fire: // red and orange LEDs flaming up and down the LED strip
        m_brightness = 0.5;
        m_speed = 0.7;
        m_toAnimate = new FireAnimation(m_brightness, m_speed, LED.LEDcount, 0.7, 0.5);
        break;
      case Larson: // a line bouncing back and forth with its width determined by size
        m_toAnimate =
            new LarsonAnimation(
                m_red, m_green, m_blue, m_white, m_speed, LED.LEDcount, BounceMode.Front, 7);
        break;
      case Rainbow: // neon cat type beat
        m_brightness = 1;
        m_toAnimate = new RainbowAnimation(m_brightness, m_speed, LED.LEDcount);
        break;
      case RgbFade: // cycling between red, greed, and blue
        m_brightness = 1;
        m_toAnimate = new RgbFadeAnimation(m_brightness, m_speed, LED.LEDcount);
        break;
      case SingleFade: // slowly turn all LEDs from solid color to off
        m_toAnimate =
            new SingleFadeAnimation(m_red, m_green, m_blue, m_white, m_speed, LED.LEDcount);
        break;
      case Strobe: // switching between solid color and full off at high speed
        m_toAnimate = new StrobeAnimation(m_red, m_green, m_blue, m_white, m_speed, LED.LEDcount);
        break;
      case Twinkle: // random LEDs turning on and off with certain color
        m_toAnimate =
            new TwinkleAnimation(
                m_red, m_green, m_blue, m_white, m_speed, LED.LEDcount, TwinklePercent.Percent6);
        break;
      case TwinkleOff: // twinkle in reverse
        m_toAnimate =
            new TwinkleOffAnimation(
                m_red,
                m_green,
                m_blue,
                m_white,
                m_speed,
                LED.LEDcount,
                TwinkleOffPercent.Percent100);
        break;
      case Solid:
        m_toAnimate = null;
        break;
      default:
        //        System.out.println("Incorrect animation type provided to changeAnimation()
        // method");
        break;
    }
  }

  // will set LEDs a coordinated color for an action
  public void expressState(SUBSYSTEM_STATES state) {
    if (state != currentRobotState) {
      switch (state) {
        case ENABLED:
          setPattern(LED.green, 0, 0, ANIMATION_TYPE.Solid); // Solid Green
          break;
        case DISABLED:
          setPattern(LED.red, 0, 0, ANIMATION_TYPE.Solid); // Solid Red
          break;
        case INTAKING:
          setPattern(LED.pink, 0, 0, ANIMATION_TYPE.Strobe); // Unknown
          break;
        case CORAL:
          setPattern(LED.yellow, 0, 0, ANIMATION_TYPE.Solid); // Solid Yellow
          break;
        case CORAL_OWNED:
          setPattern(LED.yellow, 0, 0, ANIMATION_TYPE.ColorFlow); // Flashing Yellow
          break;
        case ALGAE:
          setPattern(LED.cyan, 0, 0, ANIMATION_TYPE.Solid); // Solid Cyan
          break;
        case ALGAE_OWNED:
          setPattern(LED.cyan, 0, 0, ANIMATION_TYPE.Rainbow); // Flashing Cyan
          break;
        case REEF_LINEUP:
          setPattern(LED.blue, 0, 0, ANIMATION_TYPE.Solid); // Solid Blue
          break;
        case ENDGAME:
          setPattern(LED.white, 0, 0, ANIMATION_TYPE.Rainbow); // Rainbow
          break;
        default:
          break;
      }
      currentRobotState = state;
    }
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
