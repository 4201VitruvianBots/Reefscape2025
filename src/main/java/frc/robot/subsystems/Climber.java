// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Seconds;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicTorqueCurrentFOC;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.sim.TalonFXSimState;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.CAN;
import frc.robot.constants.CLIMBER;
import frc.robot.constants.ROBOT.CONTROL_MODE;
import org.team4201.codex.utils.CtreUtils;

public class Climber extends SubsystemBase {

  /** Creates a new climber */
  private final TalonFX climberMotor = new TalonFX(CAN.climberMotor);

  // Simulation classes help us simulate what's going on, including gravity.
  private final FlywheelSim m_climberSim =
      new FlywheelSim(
          LinearSystemId.identifyVelocitySystem(0.8, 0.6), CLIMBER.gearbox, CLIMBER.gearRatio);

  private final StatusSignal<Angle> m_positionSignal = climberMotor.getPosition().clone();
  private final StatusSignal<Voltage> m_voltageSignal = climberMotor.getMotorVoltage().clone();
  private double m_desiredPositionMeters = 0.0;
  // private boolean m_climberInitialized;
  private double m_joystickInput = 0.0;
  private CONTROL_MODE m_controlMode = CONTROL_MODE.OPEN_LOOP;
  private NeutralModeValue m_neutralMode = NeutralModeValue.Brake;
  private final MotionMagicTorqueCurrentFOC m_request = new MotionMagicTorqueCurrentFOC(0);
  private final TalonFXSimState m_motorSimState = climberMotor.getSimState();

  public Climber() {
    TalonFXConfiguration climberConfig = new TalonFXConfiguration();
    climberConfig.Slot0.kP = CLIMBER.kP;
    climberConfig.Slot0.kI = CLIMBER.kI;
    climberConfig.Slot0.kD = CLIMBER.kD;
    CtreUtils.configureTalonFx(climberMotor, climberConfig);

    climberConfig.MotionMagic.MotionMagicCruiseVelocity = 100;
    climberConfig.MotionMagic.MotionMagicAcceleration = 200;
  }

  public void holdClimber() {
    setDesiredPosition(getPulleyLengthMeters());
  }

  public void setPercentOutput(double output) {
    climberMotor.set(output);
  }

  public double getPercentOutputMotor() {
    return climberMotor.get();
  }

  public void setDesiredPosition(double desiredPosition) {
    m_desiredPositionMeters = desiredPosition;
  }

  public void setControlMode(CONTROL_MODE controlMode) {
    m_controlMode = controlMode;
  }

  public CONTROL_MODE getControlMode() {
    return m_controlMode;
  }

  public NeutralModeValue getNeutralMode() {
    return m_neutralMode;
  }

  public Double getMotorRotations() {
    m_positionSignal.refresh();
    return m_positionSignal.getValueAsDouble();
  }

  public Double getMotorVoltage() {
    m_voltageSignal.refresh();
    return m_voltageSignal.getValueAsDouble();
  }

  public double getPulleyLengthMeters() {
    return getMotorRotations() * CLIMBER.sprocketRotationsToMeters.magnitude();
  }

  // Sets the control state of the climber
  public void setClosedLoopControlMode(CONTROL_MODE mode) {
    m_controlMode = mode;
  }

  public boolean isClosedLoopControl() {
    return getControlMode() == CONTROL_MODE.CLOSED_LOOP;
  }

  public void setClimberNeutralMode(NeutralModeValue mode) {
    if (mode == m_neutralMode) return;
    m_neutralMode = mode;
    climberMotor.setNeutralMode(mode);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    switch (m_controlMode) {
      case CLOSED_LOOP:
        climberMotor.setControl(m_request.withPosition(m_desiredPositionMeters));
        break;
      case OPEN_LOOP:
      default:
        // double percentOutput = m_joystickInput * CLIMBER.kPercentOutputMultiplier;
        // setPercentOutput(percentOutput);
        break;
    }
  }

  public void simulationPeriodic() {
    m_motorSimState.setSupplyVoltage(RobotController.getBatteryVoltage());

    m_climberSim.update(0.020);

    m_motorSimState.setRawRotorPosition(m_climberSim.getAngularVelocity().times(Seconds.of(0.02)));
    m_motorSimState.setRotorVelocity(m_climberSim.getAngularVelocity().times(CLIMBER.gearRatio));
  }
}
