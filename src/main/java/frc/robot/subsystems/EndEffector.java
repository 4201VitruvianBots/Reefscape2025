// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.sim.TalonFXSimState;
import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.epilogue.NotLogged;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.CAN;
import frc.robot.constants.ENDEFFECTOR;
import frc.robot.constants.ENDEFFECTOR.ROLLERS;
import frc.robot.utils.CtreUtils;

@Logged
public class EndEffector extends SubsystemBase {
  @NotLogged private final TalonFX m_endEffectorMotor = new TalonFX(CAN.endEffectorOuttakeMotor);
  private final TalonFXSimState m_simState = m_endEffectorMotor.getSimState();
  private final DCMotorSim m_endEffectorSim =
      new DCMotorSim(
          LinearSystemId.createDCMotorSystem(ROLLERS.gearbox, ROLLERS.gearRatio, ROLLERS.kInertia),
          ROLLERS.gearbox);
  @NotLogged private final DigitalInput m_beamBreakSensor = new DigitalInput(0);

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
  }

  public void setPercentOutput(double output) {
    m_endEffectorMotor.set(output);
  }
  
  @Logged(name="Motor Output")
  public double getPercentOutput() {
    return m_endEffectorMotor.get();
  }

  @Logged(name="Has Coral")
  public boolean hasCoral() {
    return !m_beamBreakSensor.get();
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
  public void periodic() {}
}
