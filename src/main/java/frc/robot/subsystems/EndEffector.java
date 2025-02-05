// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.sim.TalonFXSimState;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.CAN;
import frc.robot.constants.ENDEFFECTOR;
import frc.robot.utils.CtreUtils;

public class EndEffector extends SubsystemBase {
  private final TalonFX m_endEffectorMotor = new TalonFX(CAN.endEffectorOuttakeMotor);
  private final StatusSignal<AngularVelocity> m_velocitySignal1 =
      m_endEffectorMotor.getVelocity().clone();
  private final StatusSignal<Voltage> m_voltageSignal1 =
      m_endEffectorMotor.getMotorVoltage().clone();
  private final StatusSignal<Current> m_currentSignal1 =
      m_endEffectorMotor.getTorqueCurrent().clone();
  private final TalonFXSimState m_endEffectorMotorSimState = m_endEffectorMotor.getSimState();

  /** Creates a new EndEffector. */
  public EndEffector() {
    TalonFXConfiguration config = new TalonFXConfiguration();
    config.Slot0.kP = ENDEFFECTOR.kP;
    config.Slot0.kI = ENDEFFECTOR.kI;
    config.Slot0.kD = ENDEFFECTOR.kD;
    config.MotorOutput.NeutralMode = NeutralModeValue.Coast;
    config.Feedback.SensorToMechanismRatio = ENDEFFECTOR.endEffectorGearRatio;
    CtreUtils.configureTalonFx(m_endEffectorMotor, config);
  }

  public void setPercentOutput(double output) {
    m_endEffectorMotor.set(output);
  }

  public AngularVelocity getSpeed() {
    return m_velocitySignal1.getValue();
  }

  public void updateLogger() {}

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
