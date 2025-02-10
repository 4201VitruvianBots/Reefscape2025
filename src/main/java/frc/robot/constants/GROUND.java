// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.constants;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Angle;

public class GROUND {
  public class PIVOT {
    public static final double canCoderOffset = -0.1396484375;

    /* Static Feedforward Gain
    This is added to the closed loop output. The sign is determined by target velocity.
    The unit for this constant is dependent on the control mode,
    typically fractional duty cycle, voltage, or torque current */
    public static final double kS = 0.05;

    /* Velocity Feedforward Gain
    The units for this gain is dependent on the control mode.
    Since this gain is multiplied by the requested velocity,
    the units should be defined as units of output per unit of requested input velocity.
    For example, when controlling velocity using a duty cycle closed loop,
    the units for the velocity feedfoward gain will be duty cycle per requested rps, or 1/rps. */
    public static final double kV = 0;

    /* Acceleration Feedforward Gain
    The units for this gain is dependent on the control mode.
    Since this gain is multiplied by the requested acceleration,
    the units should be defined as units of output per unit of requested input acceleration.
    For example, when controlling velocity using a duty cycle closed loop,
    the units for the acceleration feedforward gain will be duty cycle per requested rps/s, or 1/(rps/s). */
    public static final double kA = 0;

    /* A higher P value means you will put more effort into correcting the measured error,
    but it means you can overshoot your target and then the response will look like an oscillating graph. */
    public static final double kP = 10.0;

    /* I value is generally used to correct steady-state error
    (e.g. your goal is 100, but you a re at 99, so the sum of error
    over time will let it correct for that final error). */
    public static final double kI = 0.0;

    /* D is generally used to 'predict' the next output using the slope of the error,
    so it is usually used with P to get a fast, but accurate response. */
    public static final double kD = 0.0;

    public static final double kAccel = 50;
    public static final double kCruiseVel = 15;
    public static final double kJerk = 0;

    public enum PIVOT_SETPOINT {
      STOWED(Degrees.of(0.0)), // TODO: test setpoints
      ALGAE(Degrees.of(71.0)),
      GROUND_INTAKE(Degrees.of(60.0));

      private final Angle angle;

      PIVOT_SETPOINT(final Angle angle) {
        // this.sucks = taxes;
        this.angle = angle;
      }

      public Angle get() {
        return angle;
      }
    }

    public static final DCMotor gearBox = DCMotor.getKrakenX60(1);

    // TODO update all of these
    public static final double gearRatio = 51.0 / 1.0;

    public static final double jointLength = Units.inchesToMeters(5);
    public static final double pivotVisualizerLength = Units.inchesToMeters(17);
    public static final double pivotLength = Units.inchesToMeters(21.5);

    public static final double mass = Units.lbsToKilograms(8.0);

    public static final Angle minAngle = Degrees.of(0);

    public static final Angle maxAngle = Degrees.of(200);

    public static final Angle startingAngle = minAngle;

    public static final Angle mountingAngle = Degrees.of(0);

    public static final double maxOutput = 0.6;

    public static final double joystickMultiplier = 0.3; // For testing

    public static final boolean limitOpenLoop = false;
  }

  public class INTAKE {

    public static final double kP = 0.10; // change these
    public static final double kI = 0.00;
    public static final double kD = 0.00;
    public static final double gearRatio = 1.0 / 1.0;
    public static final DCMotor gearbox = DCMotor.getKrakenX60(1);
    public static final double kInertia = 0.001;

    public enum INTAKE_SPEED {
      INTAKING(0.5),
      HOLDING_CORAL(0.5), // TODO: update speeds later
      OUTTAKING(-0.5),
      OUTTAKE_TO_END_EFFECTOR(0.5);

      private final double value;

      INTAKE_SPEED(final double value) {
        this.value = value;
      }

      public double get() {
        return value;
      }
    }
  }
}
