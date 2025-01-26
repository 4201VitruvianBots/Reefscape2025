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
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N2;
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
import frc.robot.constants.CAGE;
import frc.robot.utils.CtreUtils;

public class CageIntake extends SubsystemBase {
  private boolean m_isIntaking = false;
  private LinearSystem<N2, N1, N2> outtakePlant = LinearSystemId.createDCMotorSystem(1, 1);
  private final TalonFX m_cageIntake = new TalonFX(CAN.cageMotor);
  private final DCMotorSim m_cageIntakeSim =
      new DCMotorSim(outtakePlant, CAGE.gearbox); 
  private double m_desiredPercentOutput;
  private final DutyCycleOut m_dutyCycleRequest = new DutyCycleOut(0);
  private double m_rpmSetpoint;
  private final VoltageOut m_voltageRequest = new VoltageOut(0);
  private final TorqueCurrentFOC m_TorqueCurrentFOC = new TorqueCurrentFOC(0);
  private final VelocityTorqueCurrentFOC m_focVelocityControl = new VelocityTorqueCurrentFOC(0);
  private TalonFXSimState m_m_cageIntakeSimState = m_cageIntake.getSimState();
  // Test mode setup
  private DoubleSubscriber m_kP_subscriber, m_kI_subscriber, m_kD_subscriber;
  private final NetworkTable coralOuttakeTab =
      NetworkTableInstance.getDefault().getTable("Shuffleboard").getSubTable("CageIntake");

  /* Creates a new CageIntake. */
  public CageIntake() {
    TalonFXConfiguration m_cageIntakeConfig = new TalonFXConfiguration();
    m_cageIntakeConfig.Slot0.kP = CAGE.kP;
    m_cageIntakeConfig.Slot0.kI = CAGE.kI;
    m_cageIntakeConfig.Slot0.kD = CAGE.kD;
    m_cageIntakeConfig.MotorOutput.NeutralMode = NeutralModeValue.Coast;
    m_cageIntakeConfig.Feedback.SensorToMechanismRatio = CAGE.gearRatio;
    m_cageIntakeConfig.ClosedLoopRamps.VoltageClosedLoopRampPeriod = 0.5;
    CtreUtils.configureTalonFx(m_cageIntake, m_cageIntakeConfig);
  }

  public void setIntakingState(boolean state) {
    m_isIntaking = state;
  }

  public boolean getIntakingState() {
    return m_isIntaking;
  }

  public void setDesiredPercentOutput(double percentOutput) {
    m_desiredPercentOutput = percentOutput;
    m_rpmSetpoint = 0;
    m_cageIntake.setControl(m_dutyCycleRequest.withOutput(percentOutput));
  }

  public void setVoltageOutput(double voltageOut) {
    m_rpmSetpoint = 0;
    m_cageIntake.setControl(m_voltageRequest.withOutput(voltageOut));
  }

  public void setFocCurrentOutput(double currentOut) {
    m_rpmSetpoint = 0;
    m_cageIntake.setControl(m_TorqueCurrentFOC.withOutput(currentOut));
  }

  public void setRPMOutputFOC(double rpm) {
    m_rpmSetpoint = rpm;

    // Phoenix 6 uses rotations per second for velocity control
    var rps = rpm / 60.0;
    m_cageIntake.setControl(m_focVelocityControl.withVelocity(rps).withFeedForward(0));
    m_cageIntake.setControl(m_focVelocityControl.withVelocity(rps).withFeedForward(0));
  }

  public void setNeutralMode(NeutralModeValue mode) {
    m_cageIntake.setNeutralMode(mode);
  }

  public void setPidValues(double v, double p, double i, double d) {
    TalonFXConfiguration config = new TalonFXConfiguration();

    // Get the current motor configs to not erase everything
    m_cageIntake.getConfigurator().refresh(config);
    config.Slot0.kV = v;
    config.Slot0.kP = p;
    config.Slot0.kI = i;
    config.Slot0.kD = d;
    CtreUtils.configureTalonFx(m_cageIntake, config);
  }

  public double getRPMsetpoint() {
    return m_rpmSetpoint;
  }

  @Override
  public void simulationPeriodic() {
    m_m_cageIntakeSimState.setSupplyVoltage(RobotController.getBatteryVoltage());
    m_cageIntakeSim.setInputVoltage(
        MathUtil.clamp(m_m_cageIntakeSimState.getMotorVoltage(), -12, 12));

    // TODO Right now this thing has no idea what time it is, so gotta update that.

    m_m_cageIntakeSimState.setRawRotorPosition(
        m_cageIntakeSim.getAngularPositionRotations() * CAGE.gearRatio);
    m_m_cageIntakeSimState.setRotorVelocity(
        m_cageIntakeSim.getAngularVelocityRPM() * CAGE.gearRatio / 60.0);
  }

  public void testInit() {
    coralOuttakeTab.getDoubleTopic("kP").publish().set(CAGE.kP);
    coralOuttakeTab.getDoubleTopic("kI").publish().set(CAGE.kI);
    coralOuttakeTab.getDoubleTopic("kD").publish().set(CAGE.kD);
    m_kP_subscriber = coralOuttakeTab.getDoubleTopic("kP").subscribe(CAGE.kP);
    m_kI_subscriber = coralOuttakeTab.getDoubleTopic("kI").subscribe(CAGE.kI);
    m_kD_subscriber = coralOuttakeTab.getDoubleTopic("kD").subscribe(CAGE.kD);
  }

  public void testPeriodic() {
    Slot0Configs slot0Configs = new Slot0Configs();
    slot0Configs.kP = m_kP_subscriber.get(CAGE.kP);
    slot0Configs.kI = m_kI_subscriber.get(CAGE.kI);
    slot0Configs.kD = m_kD_subscriber.get(CAGE.kD);
  }

  private void updateSmartDashboard() {
    SmartDashboard.putNumber("CageIntake/DesiredPercentOutput", m_desiredPercentOutput);
    SmartDashboard.putNumber("CageIntake/rpmSetpoint", m_rpmSetpoint);
  }

  @Override
  public void periodic() {
    updateSmartDashboard();
  }
}
