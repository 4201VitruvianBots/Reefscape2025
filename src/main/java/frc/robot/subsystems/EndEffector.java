// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import static edu.wpi.first.units.Units.RotationsPerSecond;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.sim.TalonFXSimState;
import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.CAN;
import frc.robot.constants.ENDEFFECTOR;
import frc.robot.constants.ENDEFFECTOR.ROLLERS;
import org.team4201.codex.utils.CtreUtils;

public class EndEffector extends SubsystemBase {
  private final TalonFX m_endEffectorMotor = new TalonFX(CAN.endEffectorOuttakeMotor);
  private final DigitalInput m_beamBreakSensor = new DigitalInput(0);

  private final StatusSignal<AngularVelocity> m_velocitySignal =
      m_endEffectorMotor.getVelocity().clone();
  private final StatusSignal<Voltage> m_voltageSignal =
      m_endEffectorMotor.getMotorVoltage().clone();
  private final StatusSignal<Current> m_supplyCurrentSignal =
      m_endEffectorMotor.getSupplyCurrent().clone();
  private final StatusSignal<Current> m_statorCurrentSignal =
      m_endEffectorMotor.getStatorCurrent().clone();
  private final StatusSignal<Current> m_torqueCurrentSignal =
      m_endEffectorMotor.getTorqueCurrent().clone();

  private final DCMotorSim m_endEffectorSim =
      new DCMotorSim(
          LinearSystemId.createDCMotorSystem(ROLLERS.gearbox, ROLLERS.kInertia, ROLLERS.gearRatio),
          ROLLERS.gearbox);
  private final TalonFXSimState m_simState = m_endEffectorMotor.getSimState();

  /** Creates a new EndEffector. */
  public EndEffector() {
    TalonFXConfiguration config = new TalonFXConfiguration();
    config.Slot0.kP = ROLLERS.kP;
    config.Slot0.kI = ROLLERS.kI;
    config.Slot0.kD = ROLLERS.kD;
    config.MotorOutput.NeutralMode = NeutralModeValue.Coast;
    config.Feedback.SensorToMechanismRatio = ROLLERS.gearRatio;
    config.MotorOutput.PeakForwardDutyCycle = ENDEFFECTOR.ROLLERS.peakForwardOutput;
    config.MotorOutput.PeakReverseDutyCycle = ENDEFFECTOR.ROLLERS.peakReverseOutput;
    CtreUtils.configureTalonFx(m_endEffectorMotor, config);

    setName("EndEffector");
    SmartDashboard.putData(this);
  }

  public void setPercentOutput(double output) {
    m_endEffectorMotor.set(output);
  }

  @Logged(name = "Motor Output", importance = Logged.Importance.INFO)
  public double getPercentOutput() {
    return m_endEffectorMotor.get();
  }

  @Logged(name = "Has Coral", importance = Logged.Importance.INFO)
  public boolean hasCoral() {
    return !m_beamBreakSensor.get() && ENDEFFECTOR.enableBeamBreak;
  }

  public AngularVelocity getVelocity() {
    if (RobotBase.isSimulation()) {
      return RotationsPerSecond.of(
          m_endEffectorSim.getAngularVelocityRPM() * ROLLERS.gearRatio / 60.0);
    } else {
      return m_velocitySignal.refresh().getValue();
    }
  }

  public Voltage getMotorVoltage() {
    return m_voltageSignal.refresh().getValue();
  }

  public Current getSupplyCurrent() {
    return m_supplyCurrentSignal.refresh().getValue();
  }

  public Current getStatorCurrent() {
    return m_statorCurrentSignal.refresh().getValue();
  }

  public Current getTorqueCurrent() {
    return m_torqueCurrentSignal.refresh().getValue();
  }

  @Override
  public void periodic() {}

  @Override
  public void simulationPeriodic() {
    m_simState.setSupplyVoltage(RobotController.getBatteryVoltage());
    m_endEffectorSim.setInputVoltage(m_simState.getMotorVoltage());

    m_endEffectorSim.update(0.02);

    m_simState.setRawRotorPosition(
        m_endEffectorSim.getAngularPositionRotations() * ROLLERS.gearRatio);
    m_simState.setRotorVelocity(
        m_endEffectorSim.getAngularVelocityRPM() * ROLLERS.gearRatio / 60.0);
  }
}
