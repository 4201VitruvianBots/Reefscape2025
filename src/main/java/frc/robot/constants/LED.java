// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.constants;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.util.Color8Bit;

/** Add your docs here. */
public class LED {
  public static final int LEDcount = 34; // TODO: change the amount of LEDs we have

  public static final double LEDstripLength = Units.inchesToMeters(7); // meters

  /** Different LED animation types */
  public enum ANIMATION_TYPE {
    ColorFlow,
    ColorFlowLong,
    Fire,
    Larson,
    Rainbow,
    RgbFade,
    SingleFade,
    Strobe,
    Twinkle,
    TwinkleOff,
    Solid
  }

  // These color are channels passed in the setPattern() method in the LED subsystem
  public static final Color8Bit red = new Color8Bit(255, 0, 0); // Disabled
  public static final Color8Bit orange = new Color8Bit(247, 116, 40);
  public static final Color8Bit yellow = new Color8Bit(150, 120, 0); // Coral Mode
  public static final Color8Bit green = new Color8Bit(0, 255, 0); // Enabled
  public static final Color8Bit cyan = new Color8Bit(24, 94, 89); // Algae Mode
  public static final Color8Bit blue = new Color8Bit(0, 0, 255); // Lined up for Reef
  public static final Color8Bit purple = new Color8Bit(128, 0, 128);
  public static final Color8Bit pink = new Color8Bit(190, 30, 35);
  public static final Color8Bit white = new Color8Bit(125, 125, 125);

  // They were in a different, probably more logical order, but I wanted them in rainbow order
  // instead.

  public enum SUBSYSTEM_STATES {
    ENDGAME,
    REEF_LINEUP,
    CORAL_OWNED,
    ALGAE_OWNED,
    CORAL,
    ALGAE,
    ENABLED,
    DISABLED;
  }
}
