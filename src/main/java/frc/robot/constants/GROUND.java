// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.constants;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.signals.GravityTypeValue;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.Mass;

public class GROUND {
  public static class PIVOT {
    public static final double canCoderOffset = -0.1396484375;

    /* Gravity Feedforward Gain
    This is added to the closed loop output to counteract the effect of gravity.
    For an arm, the actual force required is proportional to the angle of the arm,
    the CTRE library simplifies this for us by only requiring us to feed it a single value,
    which it uses to determine how much force to apply */
    public static final double kG = 0;

    /* Static Feedforward Gain
    This is added to the closed loop output. The sign is determined by target velocity.
    The unit for this constant is dependent on the control mode,
    typically fractional duty cycle, voltage, or torque current */
    public static final double kS = 0;

    /* Velocity Feedforward Gain
    The units for this gain is dependent on the control mode.
    Since this gain is multiplied by the requested velocity,
    the units should be defined as units of output per unit of requested input velocity.
    For example, when controlling velocity using a duty cycle closed loop,
    the units for the velocity feedforward gain will be duty cycle per requested rps, or 1/rps. */
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
    public static final double kP = 100.0;

    /* I value is generally used to correct steady-state error
    (e.g. your goal is 100, but you are at 99, so the sum of error
    over time will let it correct for that final error). */
    public static final double kI = 0.0;

    /* D is generally used to 'predict' the next output using the slope of the error,
    so it is usually used with P to get a fast, but accurate response. */
    public static final double kD = 1.0;
    public static final double kAccel = 480;
    public static final double kCruiseVel = 90;
    public static final double kJerk = 0;
    public static final GravityTypeValue K_GRAVITY_TYPE_VALUE = GravityTypeValue.Arm_Cosine;

    public static final DCMotor gearBox = DCMotor.getKrakenX60(1);

    // TODO update all of these
    public static final double gearRatio = 1683.0 / 32.0;

    public static final Distance jointLength = Inches.of(5);
    public static final Distance pivotVisualizerLength = Inches.of(17);
    public static final Distance pivotLength = Inches.of(21.5);

    public static final Mass mass = Pounds.of(12.0);

    public static final Angle minAngle = Degrees.of(0.0);

    public static final Angle maxAngle = Degrees.of(120.0);

    public static final Angle startingAngle = minAngle;

    public static final Angle mountingAngle = Degrees.of(0);

    public static final double maxOutput = 0.6;

    public static final double joystickMultiplier = maxOutput;

    public static final boolean limitOpenLoop = false;

    public enum SETPOINT {
      STOWED(Degrees.of(0.0)), // TODO: test setpoints
      INTAKE_ALGAE(Degrees.of(54.61)),
      INTAKE_CORAL(Degrees.of(99.765)),
      L1(Degrees.of(15));

      private final Angle angle;

      SETPOINT(final Angle angle) {
        // this.sucks = taxes;
        this.angle = angle;
      }

      public Angle get() {
        return angle;
      }
    }
  }

  public static class INTAKE {
    public static final double kP = 0.10; // change these
    public static final double kI = 0.00;
    public static final double kD = 0.00;
    public static final double gearRatio = 1.0 / 1.0;
    public static final DCMotor gearbox = DCMotor.getKrakenX60(1);
    public static final double kInertia = 0.001;

    // TODO: update speeds later
    public enum INTAKE_SPEED {
      CORAL(0.5),
      ALGAE(0.5),
      SCORE_L1(0.5),
      SCORE_ALGAE(0.5);

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
