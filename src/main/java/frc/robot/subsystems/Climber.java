// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import static edu.wpi.first.units.Units.*;

import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.CAN;
import frc.robot.constants.CLIMBER;
import frc.robot.constants.ROBOT.CONTROL_MODE;

public class Climber extends SubsystemBase {
  private final SparkMax m_climberMotor = new SparkMax(CAN.climberMotor, MotorType.kBrushless);

  private double m_desiredPositionMeters = 0.0;
  // private boolean m_climberInitialized;
  private double m_joystickInput = 0.0;
  private CONTROL_MODE m_controlMode = CONTROL_MODE.OPEN_LOOP;
  private IdleMode m_neutralMode = IdleMode.kBrake;

  private Timer m_disabledTimer = new Timer();

  // Simulation classes help us simulate what's going on
  private final DCMotorSim m_climberSim =
      new DCMotorSim(
          (LinearSystemId.createDCMotorSystem(DCMotor.getKrakenX60(1), 0.001, CLIMBER.gearRatio)),
          CLIMBER.gearbox);
  private Angle motorSimPosition = Rotations.of(0);
  private double m_buttonInput = 0.0;

  /** Creates a new climber */
  public Climber() {
    SparkMaxConfig config = new SparkMaxConfig();

    config.idleMode(m_neutralMode);

    m_climberMotor.configure(
        config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
  }

  public void holdClimber() {
    setDesiredPosition(getPulleyLengthMeters());
  }

  public void setJoystickInput(double input) {
    m_joystickInput = input;
  }

  public void setPercentOutput(double output) {
    m_climberMotor.set(output);
  }

  @Logged(name = "Motor Output", importance = Logged.Importance.INFO)
  public double getPercentOutput() {
    return m_climberMotor.get();
  }

  public void setInputVoltage(Voltage voltage) {
    m_climberMotor.setVoltage(voltage.in(Volts));
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

  public IdleMode getNeutralMode() {
    return m_neutralMode;
  }

  public Angle getMotorPosition() {
    return Rotations.of(m_climberMotor.getAbsoluteEncoder().getPosition());
  }

  @Logged(name = "Motor Rotations", importance = Logged.Importance.INFO)
  public double getMotorRotations() {
    return getMotorPosition().in(Rotations);
  }

  public Voltage getMotorVoltage() {
    return Volts.of(m_climberMotor.getBusVoltage());
  }

  public double getPulleyLengthMeters() {
    return getMotorRotations() * CLIMBER.sprocketRotationsToMeters.in(Meters);
  }

  @Logged(name = "Pulley Length Inches", importance = Logged.Importance.INFO)
  public double getPulleyLengthInches() {
    return Units.metersToInches(getPulleyLengthMeters());
  }

  public boolean isClosedLoopControl() {
    return getControlMode() == CONTROL_MODE.CLOSED_LOOP;
  }

  public void setButtonInput(double buttonInput) {
    m_buttonInput = buttonInput;
  }

  private void setNeutralMode(IdleMode neutralMode) {
    SparkMaxConfig config = new SparkMaxConfig();

    config.idleMode(neutralMode);

    m_climberMotor.configure(
        config, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);

    m_neutralMode = neutralMode;
  }

  public void teleopInit() {
    setNeutralMode(IdleMode.kBrake);
  }

  public void disabledInit() {
    m_disabledTimer.restart();
  }

  public void disabledPeriodic() {
    // Keep the climber in brake until the buzzer turns off
    // I don't wanna be the reason we lose out on an RP...
    if (m_disabledTimer.get() > 3.0 && m_neutralMode != IdleMode.kCoast)
      setNeutralMode(IdleMode.kCoast);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    switch (m_controlMode) {
      case CLOSED_LOOP:
        // climberMotor.setControl(m_request.withPosition(m_desiredPositionMeters));
        break;
      case OPEN_LOOP:
      default:
        double percentOutput = m_joystickInput * CLIMBER.kLimitedPercentOutputMultiplier;
        if (percentOutput > m_buttonInput) {
          setInputVoltage(Volts.of(percentOutput * 12.0));
        } else {
          setInputVoltage(Volts.of(m_buttonInput * 12.0));
        }
        break;
    }
  }

  public void simulationPeriodic() {
    // TODO: Update with RevLib sim
    //    m_motorSimState.setSupplyVoltage(RobotController.getBatteryVoltage());
    //    m_climberSim.setInputVoltage(m_motorSimState.getMotorVoltage());
    //
    //    m_climberSim.update(0.02);
    //
    //    motorSimPosition =
    //        motorSimPosition.plus(
    //
    // m_climberSim.getAngularVelocity().times(CLIMBER.gearRatio).times(Seconds.of(0.02)));
    //    m_motorSimState.setRawRotorPosition(motorSimPosition);
    //
    // m_motorSimState.setRotorVelocity(m_climberSim.getAngularVelocity().times(CLIMBER.gearRatio));
  }
}
