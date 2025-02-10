// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import static edu.wpi.first.units.Units.*;
import static frc.robot.constants.ENDEFFECTOR.kPivotMotionMagicAcceleration;
import static frc.robot.constants.ENDEFFECTOR.kPivotMotionMagicVelocity;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.ControlRequest;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;
import com.ctre.phoenix6.sim.CANcoderSimState;
import com.ctre.phoenix6.sim.TalonFXSimState;
import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.epilogue.NotLogged;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.CAN;
import frc.robot.constants.ENDEFFECTOR;
import frc.robot.constants.ROBOT;
import frc.robot.constants.ROBOT.CONTROL_MODE;
import frc.robot.utils.CtreUtils;

@Logged
public class EndEffectorPivot extends SubsystemBase {
  @NotLogged private final TalonFX m_pivotMotor = new TalonFX(CAN.endEffectorPivotMotor);
  @NotLogged private final CANcoder m_pivotEncoder = new CANcoder(CAN.endEffectorPivotCanCoder);

  private final NeutralModeValue m_neutralMode = NeutralModeValue.Brake;

  @NotLogged private final MotionMagicVoltage m_request = new MotionMagicVoltage(Rotations.of(0));

  @NotLogged
  private final StatusSignal<Angle> m_positionSignal = m_pivotMotor.getPosition().clone();

  @NotLogged
  private final StatusSignal<Current> m_currentSignal = m_pivotMotor.getTorqueCurrent().clone();

  private ROBOT.CONTROL_MODE m_controlMode = ROBOT.CONTROL_MODE.CLOSED_LOOP;
  private double m_joystickInput;
  private boolean m_limitJoystickInput;
  private boolean m_enforceLimits = true;
  private boolean m_userSetpoint;

  private Angle m_desiredRotation = Degrees.of(0);
  private boolean m_pivotState;

  // Simulation Code
  private final SingleJointedArmSim m_endEffectorSim =
      new SingleJointedArmSim(
          ENDEFFECTOR.pivotGearBox,
          ENDEFFECTOR.pivotGearRatio,
          SingleJointedArmSim.estimateMOI(
              ENDEFFECTOR.length.in(Meters), ENDEFFECTOR.mass.in(Kilograms)),
          ENDEFFECTOR.length.in(Meters),
          ENDEFFECTOR.minAngle.in(Radians),
          ENDEFFECTOR.maxAngle.in(Radians),
          false,
          ENDEFFECTOR.startingAngle.in(Radians));

  @NotLogged private final TalonFXSimState m_pivotMotorSimState = m_pivotMotor.getSimState();
  @NotLogged private final CANcoderSimState m_pivotEncoderSimState = m_pivotEncoder.getSimState();

  /** Creates a new EndEffectorPivot. */
  public EndEffectorPivot() {
    // Configure the CANcoder
    CANcoderConfiguration encoderConfig = new CANcoderConfiguration();
    if(RobotBase.isReal()) {
      encoderConfig.MagnetSensor.MagnetOffset = ENDEFFECTOR.encoderOffset.magnitude();
    }
    CtreUtils.configureCANCoder(m_pivotEncoder, encoderConfig);

    // Configure the Motor
    TalonFXConfiguration motorConfig = new TalonFXConfiguration();
    motorConfig.Slot0.kP = ENDEFFECTOR.kPivotP;
    motorConfig.Slot0.kI = ENDEFFECTOR.kPivotI;
    motorConfig.Slot0.kD = ENDEFFECTOR.kPivotD;
    motorConfig.MotionMagic.MotionMagicCruiseVelocity = kPivotMotionMagicVelocity;
    motorConfig.MotionMagic.MotionMagicAcceleration = kPivotMotionMagicAcceleration;
    motorConfig.MotorOutput.NeutralMode = m_neutralMode;
    motorConfig.Feedback.RotorToSensorRatio = ENDEFFECTOR.pivotGearRatio;
    motorConfig.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.RemoteCANcoder;
    motorConfig.Feedback.FeedbackRemoteSensorID = m_pivotEncoder.getDeviceID();
    CtreUtils.configureTalonFx(m_pivotMotor, motorConfig);

    setName("EndEffectorPivot");
    SmartDashboard.putData(this);
  }

  public void setState(boolean state) {
    m_pivotState = state;
  }

  public boolean getState() {
    return m_pivotState;
  }

  public void setPosition(Angle rotations) {
    if (m_enforceLimits) {
      m_desiredRotation =
          Degrees.of(
              MathUtil.clamp(
                  rotations.in(Degrees),
                  ENDEFFECTOR.minAngle.in(Degrees),
                  ENDEFFECTOR.maxAngle.in(Degrees)));

    } else {
      m_desiredRotation = rotations;
    }
  }

  public Angle getDesiredRotation() {
    return m_desiredRotation;
  }

  public void setPercentOutput(double speed) {
    m_pivotMotor.set(speed);
  }

  public double getPercentOutput() {
    return m_pivotMotor.get();
  }

  public ControlRequest getCurrentControlRequest() {
    return m_pivotMotor.getAppliedControl();
  }

  public String getCurrentControlRequestString() {
    return getCurrentControlRequest().toString();
  }

  public Angle getCurrentRotation() {
    try {
      m_positionSignal.refresh();
      return m_positionSignal.getValue();
    } catch (Exception e) {
      DriverStation.reportWarning("[EndEffectorPivot] Position signal is null!", false);
      return Degrees.of(0);
    }
  }

  // Base unit from CANcoder is in Radians
  public Angle getCANcoderAngle() {
    return m_pivotEncoder.getAbsolutePosition().getValue();
  }

  public double getCANcoderAngleDegrees() {
    return getCANcoderAngle().in(Degrees);
  }

  public void resetMotionMagicState() {
    m_desiredRotation = getCurrentRotation();
    m_pivotMotor.setControl(m_request.withPosition(m_desiredRotation));
  }

  public void zeroEncoderPosition() {
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
        m_pivotMotor.setControl(m_request.withPosition(m_desiredRotation));
        break;
      case OPEN_LOOP:
      default:
        double percentOutput = m_joystickInput * ENDEFFECTOR.kPercentOutputMultiplier;
        setPercentOutput(percentOutput);
        break;
    }
  }

  @Override
  public void simulationPeriodic() {
    m_pivotMotorSimState.setSupplyVoltage(RobotController.getBatteryVoltage());

    m_endEffectorSim.setInputVoltage(
        MathUtil.clamp(m_pivotMotorSimState.getMotorVoltage(), -12, 12));

    m_endEffectorSim.update(0.020);

    m_pivotMotorSimState.setRawRotorPosition(
        Radians.of(m_endEffectorSim.getAngleRads() * ENDEFFECTOR.pivotGearRatio));
    m_pivotMotorSimState.setRotorVelocity(
        RadiansPerSecond.of(m_endEffectorSim.getVelocityRadPerSec() * ENDEFFECTOR.pivotGearRatio));
    m_pivotEncoderSimState.setRawPosition(Radians.of(m_endEffectorSim.getAngleRads()));
    m_pivotEncoderSimState.setVelocity(
        RadiansPerSecond.of(m_endEffectorSim.getVelocityRadPerSec()));
  }
}
