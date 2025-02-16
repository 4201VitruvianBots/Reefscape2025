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
import edu.wpi.first.epilogue.NotLogged;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.units.measure.Angle;
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

@Logged
public class EndEffector extends SubsystemBase {
  private final TalonFX m_endEffectorMotor = new TalonFX(CAN.endEffectorOuttakeMotor);

  @NotLogged
  private final StatusSignal<Angle> m_positionSignal = m_endEffectorMotor.getPosition().clone();

  @NotLogged
  private final StatusSignal<AngularVelocity> m_velocitySignal =
      m_endEffectorMotor.getVelocity().clone();

  @NotLogged
  private final StatusSignal<Current> m_currentSignal =
      m_endEffectorMotor.getTorqueCurrent().clone();

  @NotLogged
  private final StatusSignal<Voltage> m_voltageSignal =
      m_endEffectorMotor.getMotorVoltage().clone();

  // Simulation Code
  @NotLogged private final TalonFXSimState m_simState = m_endEffectorMotor.getSimState();

  @NotLogged
  private final DCMotorSim m_endEffectorModel =
      new DCMotorSim(
          LinearSystemId.createDCMotorSystem(
              ENDEFFECTOR.gearbox, ENDEFFECTOR.gearRatio, ENDEFFECTOR.kInertia),
          ENDEFFECTOR.gearbox);

  /** Creates a new EndEffector. */
  public EndEffector() {
    TalonFXConfiguration config = new TalonFXConfiguration();
    config.Slot0.kP = ENDEFFECTOR.kP;
    config.Slot0.kI = ENDEFFECTOR.kI;
    config.Slot0.kD = ENDEFFECTOR.kD;
    config.MotorOutput.NeutralMode = NeutralModeValue.Coast;
    config.Feedback.SensorToMechanismRatio = ENDEFFECTOR.gearRatio;
    CtreUtils.configureTalonFx(m_endEffectorMotor, config);

    setName("EndEffector");
  }

  public void setPercentOutput(double output) {
    m_endEffectorMotor.set(output);
  }

  @Logged(name = "PercentOutput")
  public double getPercentOutput() {
    return m_endEffectorMotor.get();
  }

  @Logged(name = "Current")
  public Current getCurrent() {
    return m_currentSignal.getValue();
  }

  @Logged(name = "Voltage")
  public Voltage getVoltage() {
    return m_voltageSignal.getValue();
  }

  @Logged(name = "Angle")
  public Angle getAngle() {
    return m_positionSignal.getValue();
  }

  @Logged(name = "AngularVelocity")
  public AngularVelocity getAngularVelocity() {
    return m_velocitySignal.getValue();
  }

  public void updateLogger() {
    SmartDashboard.putNumber("EndEffector Intake/Motor Velocity", getAngularVelocity().magnitude());
    SmartDashboard.putNumber("EndEffector/Motor Output", getPercentOutput());
    SmartDashboard.putNumber("EndEffector Intake/Motor Current", getCurrent().magnitude());
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    updateLogger();
  }

  @Override
  public void simulationPeriodic() {
    m_simState.setSupplyVoltage(RobotController.getBatteryVoltage());

    m_endEffectorModel.setInputVoltage(MathUtil.clamp(m_simState.getMotorVoltage(), -12, 12));

    m_endEffectorModel.update(0.02); // TODO update this later maybe?

    m_simState.setRawRotorPosition(
        m_endEffectorModel.getAngularPositionRotations() * ENDEFFECTOR.gearRatio);
    m_simState.setRotorVelocity(
        m_endEffectorModel.getAngularVelocityRPM() * ENDEFFECTOR.gearRatio / 60.0);
  }
}
