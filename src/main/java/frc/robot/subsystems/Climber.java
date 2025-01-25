// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.CLIMBER;
import frc.robot.constants.CAN;
import frc.robot.utils.CtreUtils;

public class Climber extends SubsystemBase {

  private final TalonFX m_climberMotor = new TalonFX(CAN.climberMotor);

    public Climber() {
    TalonFXConfiguration m_climberMotorconfig = new TalonFXConfiguration();
    m_climberMotorconfig.Slot0.kP = CLIMBER.kP;
    m_climberMotorconfig.Slot0.kI = CLIMBER.kI;
    m_climberMotorconfig.Slot0.kD = CLIMBER.kD;
    m_climberMotorconfig.MotorOutput.NeutralMode = NeutralModeValue.Coast;
    m_climberMotorconfig.Feedback.SensorToMechanismRatio = CLIMBER.climberGearRatio;
    CtreUtils.configureTalonFx(m_climberMotor, m_climberMotorconfig);
  }

  public void setPercentOutput(double output) {
    m_climberMotor.set(output);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
