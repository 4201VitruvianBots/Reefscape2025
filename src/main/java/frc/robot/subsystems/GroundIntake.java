// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.sim.TalonFXSimState;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.networktables.DoubleSubscriber;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.CAN;
import frc.robot.constants.GROUND;
import frc.robot.constants.GROUND.PIVOT;
import frc.robot.utils.CtreUtils;

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
  
  // Test mode setup
  private DoubleSubscriber m_kP_subscriber,
      m_kI_subscriber,
      m_kD_subscriber,
      m_output_subscriber;
  private final NetworkTable m_groundIntakeTab =
      NetworkTableInstance.getDefault().getTable("Shuffleboard").getSubTable("GroundIntake");

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
  
  public void testInit() {
    m_groundIntakeTab.getDoubleTopic("kP").publish().set(GROUND.INTAKE.kP);
    m_groundIntakeTab.getDoubleTopic("kI").publish().set(GROUND.INTAKE.kI);
    m_groundIntakeTab.getDoubleTopic("kD").publish().set(GROUND.INTAKE.kD);

    m_groundIntakeTab.getDoubleTopic("kOutput").publish().set(m_groundIntakeMotor.get());
    
    m_kP_subscriber = m_groundIntakeTab.getDoubleTopic("kP").subscribe(GROUND.INTAKE.kP);
    m_kI_subscriber = m_groundIntakeTab.getDoubleTopic("kI").subscribe(GROUND.INTAKE.kI);
    m_kD_subscriber = m_groundIntakeTab.getDoubleTopic("kD").subscribe(GROUND.INTAKE.kD);

    m_output_subscriber =
        m_groundIntakeTab.getDoubleTopic("kOutput").subscribe(m_groundIntakeMotor.get());
  }

  public void testPeriodic() {
    Slot0Configs slot0Configs = new Slot0Configs();
    slot0Configs.kP = m_kP_subscriber.get(PIVOT.kP);
    slot0Configs.kI = m_kI_subscriber.get(PIVOT.kI);
    slot0Configs.kD = m_kD_subscriber.get(PIVOT.kD);

    m_groundIntakeMotor.getConfigurator().apply(slot0Configs);

    m_groundIntakeMotor.set(m_output_subscriber.get(m_groundIntakeMotor.get()));
  }

  @Override
  public void simulationPeriodic() {
    m_groundIntakeMotorSimState.setSupplyVoltage(RobotController.getBatteryVoltage());

    m_groundIntakeMotorSim.setInputVoltage(
        MathUtil.clamp(m_groundIntakeMotorSimState.getMotorVoltage(), -12, 12));

    m_groundIntakeMotorSim.update(0.02); // TODO update this later maybe?

    m_groundIntakeMotorSimState.setRawRotorPosition(
        m_groundIntakeMotorSim.getAngularPositionRotations() * GROUND.INTAKE.gearRatio);
    m_groundIntakeMotorSimState.setRotorVelocity(
        m_groundIntakeMotorSim.getAngularVelocityRPM() * GROUND.INTAKE.gearRatio / 60.0);
  }

  @Override
  public void periodic() {}
}
