// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.ALGAE;
import frc.robot.constants.CAN;
import frc.robot.utils.CtreUtils;

public class CageIntake extends SubsystemBase {

  private final TalonFX m_cageMotor = new TalonFX(CAN.cageMotor);

  /** Creates a new Algae. */
  public AlgaeIntake() {
    TalonFXConfiguration m_cageMotorconfig = new TalonFXConfiguration();
    m_cageMotorconfig.Slot0.kP = CAGE.kP;
    m_cageMotorconfig.Slot0.kI = CAGE.kI;
    m_cageMotorconfig.Slot0.kD = CAGE.kD;
    m_cageMotorconfig.MotorOutput.NeutralMode = NeutralModeValue.Coast;
    m_cageMotorconfig.Feedback.SensorToMechanismRatio = ALGAE.algaeGearRatio;
    CtreUtils.configureTalonFx(m_cageMotor, m_cageMotorconfig);
  }

  public void setPercentOutput(double output) {
    m_cageMotor.set(output);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
