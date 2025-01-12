// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.TorqueCurrentFOC;
import com.ctre.phoenix6.controls.VelocityTorqueCurrentFOC;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.CAN;
import frc.robot.constants.CORALOUTTAKE;
import frc.robot.utils.CtreUtils;

public class CoralOuttake extends SubsystemBase {
  private boolean m_isOuttaking = false;
  private final TalonFX outtakeMotor = new TalonFX(CAN.coralOuttakeMotor);
  // private final DCMotorSim outtakeMotorSim = new DCMotorSim(CORALOUTTAKE.Gearbox,
  // CORALOUTTAKE.gearRatio, CORALOUTTAKE.Inertia ); //TODO implement sim code
  private double m_desiredPercentOutput;
  private final DutyCycleOut m_dutyCycleRequest = new DutyCycleOut(0);
  private double m_rpmSetpoint;
  private final VoltageOut m_voltageRequest = new VoltageOut(0);
  private final TorqueCurrentFOC m_TorqueCurrentFOC = new TorqueCurrentFOC(0);
  private final VelocityTorqueCurrentFOC m_focVelocityControl = new VelocityTorqueCurrentFOC(0);

  /* Creates a new CoralOuttake. */
  public CoralOuttake() {
    TalonFXConfiguration outtakeMotorConfig = new TalonFXConfiguration();
    outtakeMotorConfig.Slot0.kP = CORALOUTTAKE.kP;
    outtakeMotorConfig.Slot0.kI = CORALOUTTAKE.kI;
    outtakeMotorConfig.Slot0.kD = CORALOUTTAKE.kD;
    outtakeMotorConfig.MotorOutput.NeutralMode = NeutralModeValue.Coast;
    outtakeMotorConfig.Feedback.SensorToMechanismRatio = CORALOUTTAKE.gearRatio;
    outtakeMotorConfig.ClosedLoopRamps.VoltageClosedLoopRampPeriod = 0.5;
    CtreUtils.configureTalonFx(outtakeMotor, outtakeMotorConfig);
  }

  public void setOuttakingState(boolean state) {
    m_isOuttaking = state;
  }

  public boolean getOuttakingState() {
    return m_isOuttaking;
  }

  public void setDesiredPercentOutput(double percentOutput) {
    m_desiredPercentOutput = percentOutput;
    m_rpmSetpoint = 0;
    outtakeMotor.setControl(m_dutyCycleRequest.withOutput(percentOutput));
  }

  public void setVoltageOutput(double voltageOut) {
    m_rpmSetpoint = 0;
    outtakeMotor.setControl(m_voltageRequest.withOutput(voltageOut));
  }

  public void setFocCurrentOutput(double currentOut) {
    m_rpmSetpoint = 0;
    outtakeMotor.setControl(m_TorqueCurrentFOC.withOutput(currentOut));
  }

  public void setRPMOutputFOC(double rpm) {
    m_rpmSetpoint = rpm;

    // Phoenix 6 uses rotations per second for velocity control
    var rps = rpm / 60.0;
    outtakeMotor.setControl(m_focVelocityControl.withVelocity(rps).withFeedForward(0));
    outtakeMotor.setControl(m_focVelocityControl.withVelocity(rps).withFeedForward(0));
  }

  public void setNeutralMode(NeutralModeValue mode) {
    outtakeMotor.setNeutralMode(mode);
  }

  public void setPidValues(double v, double p, double i, double d) {
    TalonFXConfiguration config = new TalonFXConfiguration();

    // Get the current motor configs to not erase everything
    outtakeMotor.getConfigurator().refresh(config);
    config.Slot0.kV = v;
    config.Slot0.kP = p;
    config.Slot0.kI = i;
    config.Slot0.kD = d;
    CtreUtils.configureTalonFx(outtakeMotor, config);
  }

  public double getRPMsetpoint() {
    return m_rpmSetpoint;
  }

  private void updateSmartDashboard() {
    SmartDashboard.putNumber("CoralOuttake/DesiredPercentOutput", m_desiredPercentOutput);
    SmartDashboard.putNumber("CoralOuttake/rpmSetpoint", m_rpmSetpoint);
  }
}
