// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.ENDEFFECTOR;
import frc.robot.constants.V2CAN;

public class EndEffectorWrist extends SubsystemBase {

  private final TalonFX m_pivotMotor = new TalonFX(V2CAN.endEffectorPivotMotor);
  private final CANcoder m_pivotEncoder = new CANcoder(V2CAN.endEffecotrPivotCanCoder);

  private final NeutralModeValue m_neutralMode = NeutralModeValue.Brake;

  // private final MotionMagicTorqueCurrentFOC m_request = new MotionMagicTorqueCurrentFOC();

  private double m_desiredRotations;
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

  public void setPosition(double rotations) {
    m_desiredRotations =
        MathUtil.clamp(rotations, ENDEFFECTOR.minAngleDegrees, ENDEFFECTOR.maxAngleDegrees);
  }

  public double getPosition() {
    return m_desiredRotations;
  }

  public void setPercentOutput(double speed) {
    m_pivotMotor.set(speed);
  }

  public double getPercentOutput() {
    return m_pivotMotor.get();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
