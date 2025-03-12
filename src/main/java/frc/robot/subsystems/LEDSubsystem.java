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
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color8Bit;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.CAN;
import frc.robot.constants.LED;
import frc.robot.constants.LED.*;
import frc.robot.constants.ROBOT;

public class LEDSubsystem extends SubsystemBase {
  /** Creates a new LEDSubsystem. */
  private final CANdle m_candle = new CANdle(CAN.CANdle);

  private Color8Bit m_color = new Color8Bit();
  private int m_red = 0;
  private int m_green = 0; // Setting all LED colors to none: there is no color when robot activates.
  private int m_blue = 0;
  private int m_white = 0; // We have a separate bulb in our LED strips for white.
  private double m_brightness = 0;
  private double m_speed = 0;
  private SUBSYSTEM_STATES currentRobotState = SUBSYSTEM_STATES.DISABLED;
  private boolean setSolid; // This stops the animation.
  private Animation m_toAnimate = null;

  public LEDSubsystem() {
    m_candle.configFactoryDefault();
    // Sets up LED strip.
    CANdleConfiguration configAll = new CANdleConfiguration();
    configAll.statusLedOffWhenActive = true; // Sets lights of when the LEDs are activated.
    configAll.disableWhenLOS = false; // Disables LEDs when there is no signal for control.
    configAll.stripType = LEDStripType.GRB;
    configAll.brightnessScalar =
        0.5; // 1 is highest we can go we don't want to blind everyone at the event.
    configAll.vBatOutputMode = VBatOutputMode.Modulated; // Modulate.
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

  // These will create the LED patterns.
  public void setPattern(Color8Bit color, int white, double speed, ANIMATION_TYPE toChange) {
    m_color = color;
    m_red = color.red;
    m_green = color.green;
    m_blue = color.blue;
    m_white = white;
    m_speed = speed;
    switch (toChange) {
      case ColorFlow: // Stripe of color flowing through the LED strip.
        m_toAnimate =
            new ColorFlowAnimation(
                m_red, m_green, m_blue, m_white, m_speed, LED.LEDcount, Direction.Forward);
        break;
      case ColorFlowLong: // Stripe of color flowing through the LED strip.
        m_toAnimate =
            new ColorFlowAnimation(
                m_red, m_green, m_blue, m_white, m_speed, LED.LEDcount + 15, Direction.Forward);
        break;
      case Fire: // Red and orange LEDs flaming up and down the LED strip.
        m_brightness = 0.5;
        m_speed = 0.7;
        m_toAnimate = new FireAnimation(m_brightness, m_speed, LED.LEDcount, 0.7, 0.5);
        break;
      case Larson: // A line bouncing back and forth with its width determined by size.
        m_toAnimate =
            new LarsonAnimation(
                m_red, m_green, m_blue, m_white, m_speed, LED.LEDcount, BounceMode.Front, 7);
        break;
      case Rainbow: // Neon cat type beat.
        m_brightness = 1;
        m_toAnimate = new RainbowAnimation(m_brightness, m_speed, LED.LEDcount);
        break;
      case RgbFade: // Cycling between red, greed, and blue.
        m_brightness = 1;
        m_toAnimate = new RgbFadeAnimation(m_brightness, m_speed, LED.LEDcount);
        break;
      case SingleFade: // Slowly turn all LEDs from solid color to off.
        m_toAnimate =
            new SingleFadeAnimation(m_red, m_green, m_blue, m_white, m_speed, LED.LEDcount);
        break;
      case Strobe: // switching between solid color and full off at high speed.
        m_toAnimate = new StrobeAnimation(m_red, m_green, m_blue, m_white, m_speed, LED.LEDcount);
        break;
      case Twinkle: // Random LEDs turning on and off with certain color.
        m_toAnimate =
            new TwinkleAnimation(
                m_red, m_green, m_blue, m_white, m_speed, LED.LEDcount, TwinklePercent.Percent6);
        break;
      case TwinkleOff: // Twinkle in reverse.
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

  // Will set the LEDs a coordinated color for an action.
  public void expressState(SUBSYSTEM_STATES state) {
    if (state != currentRobotState) {
      switch (state) {
        case ENABLED:
          setPattern(LED.green, 0, 0, ANIMATION_TYPE.Solid); // Solid Green
          break;
        case DISABLED:
          setPattern(LED.red, 0, 0, ANIMATION_TYPE.Solid); // Solid Red
          break;
        case CORAL:
          setPattern(LED.yellow, 0, 0, ANIMATION_TYPE.Solid); // Solid Yellow
          break;
        case CORAL_OWNED:
          setPattern(LED.yellow, 0, 0.5, ANIMATION_TYPE.Strobe); // Flashing Yellow
          break; // TODO: Adjust flashing speed.
        case ALGAE:
          setPattern(LED.cyan, 0, 0, ANIMATION_TYPE.Solid); // Solid Cyan
          break;
        case ALGAE_OWNED:
          setPattern(LED.cyan, 0, 0.5, ANIMATION_TYPE.Strobe); // Flashing Cyan
          break; // TODO: Adjust flashing speed.
        case REEF_LINEDUP:
          setPattern(LED.blue, 0, 0, ANIMATION_TYPE.Solid); // Solid Blue
          break;
        case REEF_LINEUP:
          setPattern(LED.blue, 0, 0, ANIMATION_TYPE.Strobe); // Flashing Blue
          break;
        case ENDGAME:
          setPattern(LED.white, 0, 0.5, ANIMATION_TYPE.Rainbow); // Rainbow
          break; // TODO: Adjust rainbow speed.
        default:
          break;
      }
      currentRobotState = state;
    }
  }

  public Color8Bit getColor() {
    return m_color;
  }

  private void updateSmartDashboard() {
    SmartDashboard.putString("LEDSubsystem/LED Mode", currentRobotState.toString());

    if (ROBOT.logMode.get() <= ROBOT.LOG_MODE.DEBUG.get()) {
      SmartDashboard.putNumber("LEDSubsystem/LED RED", m_color.red);
      SmartDashboard.putNumber("LEDSubsystem/LED GREEN", m_color.green);
      SmartDashboard.putNumber("LEDSubsystem/LED BLUE", m_color.blue);
      SmartDashboard.putNumber("LEDSubsystem/LED WHITE", m_white);
      SmartDashboard.putNumber("LEDSubsystem/LED SPEED", m_speed);
    }
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    if (m_toAnimate == null && !setSolid) {
      setSolid = true;
      m_candle.setLEDs(m_red, m_green, m_blue, 0, 0, LED.LEDcount); // setting all LEDs to color
    } else {
      setSolid = false;
      m_candle.animate(m_toAnimate); // setting the candle animation to m_animation if not null
    }
    SmartDashboard.putString("LED Mode", currentRobotState.toString());

    if (ROBOT.logMode.get() <= ROBOT.LOG_MODE.NORMAL.get()) updateSmartDashboard();
  }
}
