// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.sim.TalonFXSimState;
import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.CAN;
import frc.robot.constants.ENDEFFECTOR.ROLLERS;
import frc.robot.utils.CtreUtils;

@Logged
public class EndEffector extends SubsystemBase {
  private final TalonFX m_endEffectorMotor = new TalonFX(CAN.endEffectorOuttakeMotor);
  private final StatusSignal<AngularVelocity> m_velocitySignal =
      m_endEffectorMotor.getVelocity().clone();
  private final StatusSignal<Voltage> m_voltageSignal =
      m_endEffectorMotor.getMotorVoltage().clone();
  private final StatusSignal<Current> m_currentSignal =
      m_endEffectorMotor.getTorqueCurrent().clone();
  private final TalonFXSimState m_simState = m_endEffectorMotor.getSimState();
  private final DCMotorSim m_endEffectorSim =
      new DCMotorSim(
          LinearSystemId.createDCMotorSystem(
              ROLLERS.gearbox, ROLLERS.gearRatio, ROLLERS.kInertia),
          ROLLERS.gearbox);

  /** Creates a new EndEffector. */
  public EndEffector() {
    TalonFXConfiguration config = new TalonFXConfiguration();
    config.Slot0.kP = ROLLERS.kP;
    config.Slot0.kI = ROLLERS.kI;
    config.Slot0.kD = ROLLERS.kD;
    config.MotorOutput.NeutralMode = NeutralModeValue.Coast;
    config.Feedback.SensorToMechanismRatio = ROLLERS.gearRatio;
    CtreUtils.configureTalonFx(m_endEffectorMotor, config);

    setName("EndEffector");
  }

  public void setPercentOutput(double output) {
    m_endEffectorMotor.set(output);
  }

  public void updateLogger() {
    SmartDashboard.putNumber(
        "EndEffector/Motor Velocity", m_velocitySignal.getValueAsDouble());
    SmartDashboard.putNumber("EndEffector/Motor Output", m_voltageSignal.getValueAsDouble() / 12.0);
    SmartDashboard.putNumber(
        "EndEffector/Motor Current", m_currentSignal.getValueAsDouble());
  }

  @Override
  public void simulationPeriodic() {
    m_simState.setSupplyVoltage(RobotController.getBatteryVoltage());

    m_endEffectorSim.setInputVoltage(MathUtil.clamp(m_simState.getMotorVoltage(), -12, 12));

    m_endEffectorSim.update(0.02); // TODO update this later maybe?

    m_simState.setRawRotorPosition(
        m_endEffectorSim.getAngularPositionRotations() * ROLLERS.gearRatio);
    m_simState.setRotorVelocity(
        m_endEffectorSim.getAngularVelocityRPM() * ROLLERS.gearRatio / 60.0);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    updateLogger();
  }
}
