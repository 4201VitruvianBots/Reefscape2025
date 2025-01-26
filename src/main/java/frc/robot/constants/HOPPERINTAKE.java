// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.constants;

/** Add your docs here. */
public class HOPPERINTAKE {

    public enum INTAKE_SPEEDS {

        INTAKING(0.5, 0.5),
        HOLDING_CORAL(0.5, 0.5); // TODO: update speeds later

        private final double value1;
        private final double value2;
        
        INTAKE_SPEEDS(final double value1, final double value2) {
          this.value1 = value1;
          this.value2 = value2;
        }
        
        public double get1() {
            return value1;
        }
        public double get2() {
            return value2;
        }

    }
}
