// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.TorqueCurrentFOC;
import com.ctre.phoenix6.controls.VelocityTorqueCurrentFOC;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.sim.TalonFXSimState;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.system.LinearSystem;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.networktables.DoubleSubscriber;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.CAN;
import frc.robot.constants.CORALOUTTAKE;
import frc.robot.utils.CtreUtils;

public class CoralOuttake extends SubsystemBase {
  private boolean m_isOuttaking = false;
  private LinearSystem outtakePlant = LinearSystemId.createDCMotorSystem(1,1);
  private final TalonFX outtakeMotor = new TalonFX(CAN.outtakeMotor);
  private final DCMotorSim outtakeMotorSim = 
    new DCMotorSim(outtakePlant, CORALOUTTAKE.Gearbox); //TODO implement sim code
  private double m_desiredPercentOutput;
  private final DutyCycleOut m_dutyCycleRequest = new DutyCycleOut(0);
  private double m_rpmSetpoint;
  private final VoltageOut m_voltageRequest = new VoltageOut(0);
  private final TorqueCurrentFOC m_TorqueCurrentFOC = new TorqueCurrentFOC(0);
  private final VelocityTorqueCurrentFOC m_focVelocityControl = new VelocityTorqueCurrentFOC(0);
  private TalonFXSimState m_outtakeMotorSimState = outtakeMotor.getSimState();
  // Test mode setup
  private DoubleSubscriber m_kP_subscriber, m_kI_subscriber, m_kD_subscriber;
  private final NetworkTable coralOuttakeTab =
      NetworkTableInstance.getDefault().getTable("Shuffleboard").getSubTable("CoralOuttake");

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

  @Override
  public void simulationPeriodic() {
    m_outtakeMotorSimState.setSupplyVoltage(RobotController.getBatteryVoltage());
    outtakeMotorSim.setInputVoltage(MathUtil.clamp(m_outtakeMotorSimState.getMotorVoltage(), -12, 12));

    //TODO Right now this thing has no idea what time it is, so gotta update that.

    m_outtakeMotorSimState.setRawRotorPosition(
      outtakeMotorSim.getAngularPositionRotations() * CORALOUTTAKE.gearRatio);
    m_outtakeMotorSimState.setRotorVelocity(
      outtakeMotorSim.getAngularVelocityRPM() * CORALOUTTAKE.gearRatio / 60.0);
  }

  public void testInit() {
    coralOuttakeTab.getDoubleTopic("kP").publish().set(CORALOUTTAKE.kP);
    coralOuttakeTab.getDoubleTopic("kI").publish().set(CORALOUTTAKE.kI);
    coralOuttakeTab.getDoubleTopic("kD").publish().set(CORALOUTTAKE.kD);
    m_kP_subscriber = coralOuttakeTab.getDoubleTopic("kP").subscribe(CORALOUTTAKE.kP);
    m_kI_subscriber = coralOuttakeTab.getDoubleTopic("kI").subscribe(CORALOUTTAKE.kI);
    m_kD_subscriber = coralOuttakeTab.getDoubleTopic("kD").subscribe(CORALOUTTAKE.kD);
  }

  public void testPeriodic() {
    Slot0Configs slot0Configs = new Slot0Configs();
    slot0Configs.kP = m_kP_subscriber.get(CORALOUTTAKE.kP);
    slot0Configs.kI = m_kI_subscriber.get(CORALOUTTAKE.kI);
    slot0Configs.kD = m_kD_subscriber.get(CORALOUTTAKE.kD);
  }
  

  private void updateSmartDashboard() {
    SmartDashboard.putNumber("CoralOuttake/DesiredPercentOutput", m_desiredPercentOutput);
    SmartDashboard.putNumber("CoralOuttake/rpmSetpoint", m_rpmSetpoint);
  }
}
