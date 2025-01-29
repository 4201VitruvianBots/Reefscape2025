// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.constants;

/** Add your docs here. */
public class HOPPERINTAKE {

    public static final double kP = 0.10; //change these
    public static final double kI = 0.00;
    public static final double kD = 0.00;
    public static final double gearRatio = 1.0 / 1.0;

    public enum INTAKE_SPEED {

        INTAKING(0.5),
        HOLDING_CORAL(0.5); // TODO: update speeds later

        private final double value;

        INTAKE_SPEED(final double value) {
          this.value = value;
        }

        public double get() {
            return value;
        }

    }
}
