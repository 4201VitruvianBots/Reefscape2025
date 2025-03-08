// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.sim.TalonFXSimState;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.CAN;
import frc.robot.constants.GROUND;
import org.team4201.codex.utils.CtreUtils;

public class GroundIntake extends SubsystemBase {

  private final TalonFX m_groundIntakeMotor = new TalonFX(CAN.groundRollerMotor);
  private final StatusSignal<AngularVelocity> m_velocitySignal =
      m_groundIntakeMotor.getVelocity().clone();
  private final StatusSignal<Voltage> m_voltageSignal =
      m_groundIntakeMotor.getMotorVoltage().clone();
  private final StatusSignal<Current> m_currentSignal =
      m_groundIntakeMotor.getTorqueCurrent().clone();
  private final TalonFXSimState m_groundIntakeMotorSimState = m_groundIntakeMotor.getSimState();
  private final DCMotorSim m_groundIntakeMotorSim =
      new DCMotorSim(
          LinearSystemId.createDCMotorSystem(
              GROUND.INTAKE.gearbox, GROUND.INTAKE.gearRatio, GROUND.INTAKE.kInertia),
          GROUND.INTAKE.gearbox);

  public GroundIntake() {
    TalonFXConfiguration config = new TalonFXConfiguration();
    config.Slot0.kP = GROUND.INTAKE.kP;
    config.Slot0.kI = GROUND.INTAKE.kI;
    config.Slot0.kD = GROUND.INTAKE.kD;
    config.Feedback.SensorToMechanismRatio = GROUND.INTAKE.gearRatio;
    config.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    config.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
    CtreUtils.configureTalonFx(m_groundIntakeMotor, config);
  }
  
  public void setPercentOutput(double speed) {
    m_groundIntakeMotor.set(speed);
  }

  public void updateLogger() {
    SmartDashboard.putNumber("Ground Intake/Motor Velocity", m_velocitySignal.getValueAsDouble());
    SmartDashboard.putNumber(
        "Ground Intake/Motor Output", m_voltageSignal.getValueAsDouble() / 12.0);
    SmartDashboard.putNumber("Ground Intake/Motor Current", m_currentSignal.getValueAsDouble());
  }

  public boolean isConnected() {
    return m_groundIntakeMotor.isConnected();
  }
  
  @Override
  public void simulationPeriodic() {
    m_groundIntakeMotorSimState.setSupplyVoltage(RobotController.getBatteryVoltage());

    m_groundIntakeMotorSim.setInputVoltage(m_groundIntakeMotorSimState.getMotorVoltage());

    m_groundIntakeMotorSim.update(0.02);

    m_groundIntakeMotorSimState.setRawRotorPosition(
        m_groundIntakeMotorSim.getAngularPositionRotations() * GROUND.INTAKE.gearRatio);
    m_groundIntakeMotorSimState.setRotorVelocity(
        m_groundIntakeMotorSim.getAngularVelocityRPM() * GROUND.INTAKE.gearRatio / 60.0);
  }

  @Override
  public void periodic() {}
}
