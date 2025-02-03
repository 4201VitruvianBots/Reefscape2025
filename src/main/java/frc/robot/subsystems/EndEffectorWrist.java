// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Degrees;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicTorqueCurrentFOC;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.CAN;
import frc.robot.constants.ENDEFFECTOR;
import frc.robot.constants.ROBOT;
import frc.robot.constants.ROBOT.CONTROL_MODE;

public class EndEffectorWrist extends SubsystemBase {

  private final TalonFX m_pivotMotor = new TalonFX(CAN.endEffectorPivotMotor);
  private final CANcoder m_pivotEncoder = new CANcoder(CAN.endEffectorPivotCanCoder);

  private final NeutralModeValue m_neutralMode = NeutralModeValue.Brake;

  private final MotionMagicTorqueCurrentFOC m_request =
      new MotionMagicTorqueCurrentFOC(getCurrentRotation());

  private final StatusSignal<Angle> m_positionSignal = m_pivotMotor.getPosition().clone();
  private final StatusSignal<Current> m_currentSignal = m_pivotMotor.getTorqueCurrent().clone();

  private ROBOT.CONTROL_MODE m_controlMode = ROBOT.CONTROL_MODE.CLOSED_LOOP;
  private double m_joystickInput;
  private boolean m_limitJoystickInput;
  private boolean m_enforceLimits;
  private boolean m_userSetpoint;

  private Angle m_desiredRotations;
  private boolean m_pivotState;

  /** Creates a nepw EndEffectorWrist. */
  public EndEffectorWrist() {
    TalonFXConfiguration configuration = new TalonFXConfiguration();
    configuration.Slot0.kP = ENDEFFECTOR.kP;
    configuration.Slot0.kI = ENDEFFECTOR.kI;
    configuration.Slot0.kD = ENDEFFECTOR.kD;
    configuration.MotorOutput.NeutralMode = m_neutralMode;
    configuration.Feedback.RotorToSensorRatio = ENDEFFECTOR.endEffectorPivotGearRatio;
  }

  public void setState(boolean state) {
    m_pivotState = state;
  }

  public boolean getState() {
    return m_pivotState;
  }

  public void setPosition(Angle rotations) {
    m_desiredRotations =
        Degrees.of(
            MathUtil.clamp(
                rotations.in(Degrees),
                ENDEFFECTOR.minAngle.in(Degrees),
                ENDEFFECTOR.maxAngle.in(Degrees)));
  }

  public Angle getPosition() {
    return m_desiredRotations;
  }

  public void setPercentOutput(double speed) {
    m_pivotMotor.set(speed);
  }

  public double getPercentOutput() {
    return m_pivotMotor.get();
  }

  public Angle getCurrentRotation() {
    m_positionSignal.refresh();
    return m_positionSignal.getValue();
  }

  public double getCANcoderAngle() {
    return m_pivotEncoder.getAbsolutePosition().getValueAsDouble() * 360;
  }

  public void resetMotionMagicState() {
    m_desiredRotations = getCurrentRotation();
    m_pivotMotor.setControl(m_request.withPosition(m_desiredRotations));
  }

  public void resetEncoderPosition() {
    resetEncoderPosition(ENDEFFECTOR.startingAngle);
  }

  public void resetEncoderPosition(Angle angle) {
    m_pivotMotor.setPosition(angle);
    resetMotionMagicState();
  }

  public void setControlMode(CONTROL_MODE mode) {
    m_controlMode = mode;
  }

  public CONTROL_MODE getControlMode() {
    return m_controlMode;
  }

  public void setJoystickLimit(boolean limit) {
    m_limitJoystickInput = limit;
  }

  public void setJoystickY(double m_joystickY) {
    m_joystickInput = m_joystickY;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    switch (m_controlMode) {
      case CLOSED_LOOP:
        m_pivotMotor.setControl(m_request.withPosition(m_desiredRotations));
        break;
      case OPEN_LOOP:
      default:
        double percentOutput = m_joystickInput * ENDEFFECTOR.kPercentOutputMultiplier;
        setPercentOutput(percentOutput);
        break;
    }
  }
}
