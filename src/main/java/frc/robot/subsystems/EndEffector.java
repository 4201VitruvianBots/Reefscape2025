// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.configs.VoltageConfigs;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.sim.TalonFXSimState;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.system.LinearSystem;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.CAN;
import frc.robot.constants.ENDEFFECTOR;
import frc.robot.utils.CtreUtils;

public class EndEffector extends SubsystemBase {
private final TalonFX m_endEffector = new TalonFX(CAN.endEffectorOuttakeMotor);
private final StatusSignal <AngularVelocity> m_velocitySignal1 = 
m_endEffector.getVelocity().clone();
private final StatusSignal<Voltage> m_voltageSignal1 =
m_endEffector.getMotorVoltage().clone();
  private final StatusSignal<Current> m_currentSignal1 =
  m_endEffector.getTorqueCurrent().clone();
    private final TalonFXSimState m_endEffectorSim =
    m_endEffector.getSimState();
private final DCMotorSim m_endEffectorsim = 
new DCMotorSim(
  LinearSystemId.createDCMotorSystem(
    ENDEFFECTOR.GearRatio, ENDEFFECTOR.GearRatio, ENDEFFECTOR.Inertia
  ), ENDEFFECTOR.GearRatio
);

  /** Creates a new EndEffector. */
  public EndEffector() {
    TalonFXConfiguration m_endEffectorMotorconfig = new TalonFXConfiguration();
    m_endEffectorMotorconfig.Slot0.kP = ENDEFFECTOR.kP;
    m_endEffectorMotorconfig.Slot0.kI = ENDEFFECTOR.kI;
    m_endEffectorMotorconfig.Slot0.kD = ENDEFFECTOR.kD;
    m_endEffectorMotorconfig.MotorOutput.NeutralMode = NeutralModeValue.Coast;
    m_endEffectorMotorconfig.Feedback.SensorToMechanismRatio = ENDEFFECTOR.GearRatio;
    CtreUtils.configureTalonFx(m_endEffector, m_endEffectorMotorconfig);

  }

  public void setPercentOutput(double output) {
    m_endEffector.set(output);
  }
  public void updateLogger() {
    SmartDashboard.putNumber("Endeffector Intake/Motor", m_velocitySignal1.getValueAsDouble());
    SmartDashboard.putNumber(
        "Endeffector/Motor1 Output", m_voltageSignal1.getValueAsDouble() / 12.0);
    SmartDashboard.putNumber("EndEffector Intake/Motor1 Current", m_currentSignal1.getValueAsDouble());
  }
  

  public void updatelogger() {}

  
  @Override
  public void simulationPeriodic() {
    m_endEffectorSim.setSupplyVoltage(RobotController.getBatteryVoltage());

    m_endEffectorsim.setInputVoltage(
        MathUtil.clamp(m_endEffectorSim.getMotorVoltage(), -12, 12));

        m_endEffectorsim.update(0.02); // TODO update this later maybe?

        m_endEffectorSim.setRawRotorPosition(
          m_endEffectorsim.getAngularPositionRotations() * ENDEFFECTOR.GearRatio);
          m_endEffectorSim.setRotorVelocity(
            m_endEffectorsim.getAngularVelocityRPM() * ENDEFFECTOR.GearRatio / 60.0);
  }

  

  
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
