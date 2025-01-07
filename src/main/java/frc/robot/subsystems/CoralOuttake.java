// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.CAN;
import frc.robot.constants.CORALOUTTAKE;
import frc.robot.utils.CtreUtils;
public class CoralOuttake extends SubsystemBase {
  private boolean m_isOuttaking = false;
  private final TalonFX outtakeMotor = new TalonFX(CAN.outtakeMotor);

  /** Creates a new CoralOuttake. */
  public CoralOuttake() {
    TalonFXConfiguration outtakeMotorConfig = new TalonFXConfiguration();
    outtakeMotorConfig.Slot0.kP = CORALOUTTAKE.kP;
    outtakeMotorConfig.Slot0.kI = CORALOUTTAKE.kI;
    outtakeMotorConfig.Slot0.kD = CORALOUTTAKE.kD;
    outtakeMotorConfig.MotorOutput.NeutralMode = NeutralModeValue.Coast;
    outtakeMotorConfig.Feedback.SensorToMechanismRatio = CORALOUTTAKE.gearRatio;
    outtakeMotorConfig.ClosedLoopRamps.VoltageClosedLoopRampPeriod = 0.5;
    CtreUtils.configureTalonFx(outtakeMotor, outtakeMotorConfig);
  }

  /**
   * Example command factory method.
   *
   * @return a command
   */

  /**
   * An example method querying a boolean state of the subsystem (for example, a digital sensor).
   *
   * @return value of some boolean subsystem state, such as a digital sensor.
   */
  public boolean exampleCondition() {
    // Query some boolean state, such as a digital sensor.
    return false;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}
