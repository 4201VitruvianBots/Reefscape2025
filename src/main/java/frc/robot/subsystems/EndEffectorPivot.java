// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import static edu.wpi.first.units.Units.*;

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
import frc.robot.constants.ENDEFFECTOR.PIVOT;
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
  private boolean m_userSetpoint;

  private Angle m_desiredRotation = Degrees.of(0);
  private boolean m_pivotState;

  // Simulation Code
  private final SingleJointedArmSim m_endEffectorSim =
      new SingleJointedArmSim(
          PIVOT.pivotGearBox,
          PIVOT.pivotGearRatio,
          SingleJointedArmSim.estimateMOI(PIVOT.length.in(Meters), PIVOT.mass.in(Kilograms)),
          PIVOT.length.in(Meters),
          PIVOT.minAngle.in(Radians),
          PIVOT.maxAngle.in(Radians),
          false,
          PIVOT.startingAngle.in(Radians));

  @NotLogged private final TalonFXSimState m_pivotMotorSimState = m_pivotMotor.getSimState();
  @NotLogged private final CANcoderSimState m_pivotEncoderSimState = m_pivotEncoder.getSimState();

  /** Creates a new EndEffectorPivot. */
  public EndEffectorPivot() {
    // Configure the Motor
    TalonFXConfiguration motorConfig = new TalonFXConfiguration();
    if (RobotBase.isReal()) {
      motorConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
    } else {
      motorConfig.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
    }
    motorConfig.Slot0.kP = PIVOT.kPivotP;
    motorConfig.Slot0.kI = PIVOT.kPivotI;
    motorConfig.Slot0.kD = PIVOT.kPivotD;
    motorConfig.Slot1.kP = PIVOT.kPivotP;
    motorConfig.Slot1.kI = PIVOT.kPivotI;
    motorConfig.Slot1.kD = PIVOT.kPivotD;
    motorConfig.Slot1.kG = PIVOT.kGPositive;
    motorConfig.Slot0.GravityType = PIVOT.K_GRAVITY_TYPE_VALUE;
    motorConfig.MotionMagic.MotionMagicCruiseVelocity = PIVOT.kPivotMotionMagicVelocity;
    motorConfig.MotionMagic.MotionMagicAcceleration = PIVOT.kPivotMotionMagicAcceleration;
    motorConfig.MotorOutput.NeutralMode = m_neutralMode;
    motorConfig.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.RemoteCANcoder;
    if (motorConfig.Feedback.FeedbackSensorSource == FeedbackSensorSourceValue.RotorSensor) {
      // For internal TalonFX Sensor
      motorConfig.Feedback.SensorToMechanismRatio = PIVOT.pivotGearRatio;
    } else {
      // For RemoteCANcoder/SyncCANcoder/FusedCANcoder
      motorConfig.Feedback.RotorToSensorRatio = PIVOT.pivotGearRatio;
      motorConfig.Feedback.FeedbackRemoteSensorID = m_pivotEncoder.getDeviceID();
    }
    CtreUtils.configureTalonFx(m_pivotMotor, motorConfig);

    // Configure the CANcoder
    CANcoderConfiguration encoderConfig = new CANcoderConfiguration();
    if (RobotBase.isReal()) {
      encoderConfig.MagnetSensor.MagnetOffset = PIVOT.encoderOffset.magnitude();
      encoderConfig.MagnetSensor.SensorDirection = SensorDirectionValue.CounterClockwise_Positive;
    }
    CtreUtils.configureCANCoder(m_pivotEncoder, encoderConfig);

    m_pivotMotor.setPosition(getCANcoderAngle());

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
    if (PIVOT.enforceLimits) {
      m_desiredRotation =
          Degrees.of(
              MathUtil.clamp(
                  rotations.in(Degrees), PIVOT.minAngle.in(Degrees), PIVOT.maxAngle.in(Degrees)));

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

  public Angle getMotorAngle() {
    return m_pivotMotor.getPosition().getValue();
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
    resetEncoderPosition(PIVOT.startingAngle);
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

  private void updateSmartDashboard() {
    SmartDashboard.putNumber("End Effector Pivot/End Effector Angle", getCANcoderAngleDegrees());
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
        double percentOutput = m_joystickInput * PIVOT.kLimitedPercentOutputMultiplier;
        setPercentOutput(percentOutput);
        break;
    }
    updateSmartDashboard();
  }

  @Override
  public void simulationPeriodic() {
    m_pivotMotorSimState.setSupplyVoltage(RobotController.getBatteryVoltage());

    m_endEffectorSim.setInputVoltage(
        MathUtil.clamp(m_pivotMotorSimState.getMotorVoltage(), -12, 12));

    m_endEffectorSim.update(0.020);

    // Update the pivotMotor simState
    m_pivotMotorSimState.setRawRotorPosition(
        Radians.of(m_endEffectorSim.getAngleRads() * PIVOT.pivotGearRatio));
    m_pivotMotorSimState.setRotorVelocity(
        RadiansPerSecond.of(m_endEffectorSim.getVelocityRadPerSec() * PIVOT.pivotGearRatio));

    // Update the pivotEncoder simState
    m_pivotEncoderSimState.setRawPosition(Radians.of(m_endEffectorSim.getAngleRads()));
    m_pivotEncoderSimState.setVelocity(
        RadiansPerSecond.of(m_endEffectorSim.getVelocityRadPerSec()));
  }
}
