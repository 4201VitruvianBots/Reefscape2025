// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.constants;

import edu.wpi.first.math.system.plant.DCMotor;

/** Add your docs here. */
public class HOPPERINTAKE {

  public static final double kP = 1.0; // change these
  public static final double kI = 0.00;
  public static final double kD = 0.00;
  public static final double gearRatio = 1.0 / 1.0;
  public static final DCMotor gearbox = DCMotor.getKrakenX60(1);
  public static final double kInertia = 0.001;
  public static final double peakForwardOutput = 0.6;
  public static final double peakReverseOutput = -0.4;
  public static final double hopperIntakeBeamBreakTimeout = 0.08;

  public enum INTAKE_SPEED {
    INTAKING(0.6),
    ZERO(0),
    FREEING_CORAL(-0.1); // TODO: update speeds later

    private final double value;

    INTAKE_SPEED(final double value) {
      this.value = value;
    }

    public double get() {
      return value;
    }
  }
}
