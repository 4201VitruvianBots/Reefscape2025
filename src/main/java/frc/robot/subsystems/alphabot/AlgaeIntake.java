// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.alphabot;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.alphabot.ALGAE;
import frc.robot.constants.alphabot.CAN;
import org.team4201.codex.utils.CtreUtils;

public class AlgaeIntake extends SubsystemBase {

  private final TalonFX m_algaeMotor = new TalonFX(CAN.algaeMotor);

  /** Creates a new Algae. */
  public AlgaeIntake() {
    TalonFXConfiguration m_algaeMotorConfig = new TalonFXConfiguration();
    m_algaeMotorConfig.Slot0.kP = ALGAE.kP;
    m_algaeMotorConfig.Slot0.kI = ALGAE.kI;
    m_algaeMotorConfig.Slot0.kD = ALGAE.kD;
    m_algaeMotorConfig.MotorOutput.NeutralMode = NeutralModeValue.Coast;
    m_algaeMotorConfig.Feedback.SensorToMechanismRatio = ALGAE.algaeGearRatio;
    CtreUtils.configureTalonFx(m_algaeMotor, m_algaeMotorConfig);
  }

  public void setPercentOutput(double output) {
    m_algaeMotor.set(output);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
