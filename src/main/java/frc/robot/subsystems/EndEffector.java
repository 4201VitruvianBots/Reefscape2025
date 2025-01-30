// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.CAN;
import frc.robot.constants.ENDEFFECTOR;
import frc.robot.utils.CtreUtils;

public class EndEffector extends SubsystemBase {
  private final TalonFX m_endEffectorMotor = new TalonFX(CAN.endEffectorOuttakeMotor);

  /** Creates a new EndEffector. */
  public EndEffector() {
    TalonFXConfiguration m_endEffectorMotorconfig = new TalonFXConfiguration();
    m_endEffectorMotorconfig.Slot0.kP = ENDEFFECTOR.kP;
    m_endEffectorMotorconfig.Slot0.kI = ENDEFFECTOR.kI;
    m_endEffectorMotorconfig.Slot0.kD = ENDEFFECTOR.kD;
    m_endEffectorMotorconfig.MotorOutput.NeutralMode = NeutralModeValue.Coast;
    m_endEffectorMotorconfig.Feedback.SensorToMechanismRatio = ENDEFFECTOR.endEffectorGearRatio;
    CtreUtils.configureTalonFx(m_endEffectorMotor, m_endEffectorMotorconfig);
  }

  public void setPercentOutput(double output) {
    m_endEffectorMotor.set(output);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
