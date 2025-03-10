// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicTorqueCurrentFOC;
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
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.networktables.DoubleSubscriber;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.units.measure.*;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.CAN;
import frc.robot.constants.ELEVATOR;
import frc.robot.constants.ENDEFFECTOR.PIVOT;
import frc.robot.constants.ENDEFFECTOR.PIVOT.PIVOT_SETPOINT;
import frc.robot.constants.ROBOT;
import frc.robot.constants.ROBOT.CONTROL_MODE;
import org.team4201.codex.utils.CtreUtils;

public class EndEffectorPivot extends SubsystemBase {
  private final TalonFX m_pivotMotor = new TalonFX(CAN.endEffectorPivotMotor);
  private final CANcoder m_pivotEncoder = new CANcoder(CAN.endEffectorPivotCanCoder);

  @Logged(name = "Neutral Mode", importance = Logged.Importance.INFO)
  private final NeutralModeValue m_neutralMode = NeutralModeValue.Brake;

  private final MotionMagicTorqueCurrentFOC m_request = new MotionMagicTorqueCurrentFOC(Rotations.of(0));

  private final StatusSignal<Angle> m_positionSignal = m_pivotMotor.getPosition().clone();
  private final StatusSignal<AngularVelocity> m_velocitySignal = m_pivotMotor.getVelocity().clone();
  private final StatusSignal<AngularAcceleration> m_accelSignal =
      m_pivotMotor.getAcceleration().clone();
  private final StatusSignal<Voltage> m_voltageSignal = m_pivotMotor.getMotorVoltage().clone();
  private final StatusSignal<Current> m_supplyCurrentSignal =
      m_pivotMotor.getSupplyCurrent().clone();
  private final StatusSignal<Current> m_statorCurrentSignal =
      m_pivotMotor.getStatorCurrent().clone();
  private final StatusSignal<Current> m_torqueCurrentSignal =
      m_pivotMotor.getTorqueCurrent().clone();

  private final StatusSignal<Angle> m_canCoderAbsolutePositionSignal =
      m_pivotEncoder.getAbsolutePosition().clone();

  @Logged(name = "Control Mode", importance = Logged.Importance.INFO)
  ROBOT.CONTROL_MODE m_controlMode = ROBOT.CONTROL_MODE.CLOSED_LOOP;

  @Logged(name = "Joystick Input", importance = Logged.Importance.DEBUG)
  private double m_joystickInput = 0.0;

  private Angle m_desiredRotation = PIVOT_SETPOINT.STOWED.get();

  // Simulation Code
  private final SingleJointedArmSim m_endEffectorSim =
      new SingleJointedArmSim(
          PIVOT.pivotGearBox,
          PIVOT.pivotGearRatio,
          SingleJointedArmSim.estimateMOI(PIVOT.baseLength.in(Meters), PIVOT.mass.in(Kilograms)),
          PIVOT.baseLength.in(Meters),
          PIVOT.minAngle.in(Radians),
          PIVOT.maxAngle.in(Radians),
          false,
          PIVOT.startingAngle.in(Radians));

  private final TalonFXSimState m_pivotMotorSimState = m_pivotMotor.getSimState();
  private final CANcoderSimState m_pivotEncoderSimState = m_pivotEncoder.getSimState();

  private DoubleSubscriber
      m_kG_subscriber,
      m_kS_subscriber,
      m_kV_Subscriber,
      m_kA_Subscriber,
      m_kP_subscriber,
      m_kI_subscriber,
      m_kD_subscriber,
      m_velocitySubscriber,
      m_accelerationSubscriber,
      m_setpointSubscriber;
  private final NetworkTable endEffectorPivotTab =
      NetworkTableInstance.getDefault().getTable("Shuffleboard").getSubTable("EndEffectorPivot");
      
  /** Creates a new EndEffectorPivot. */
  public EndEffectorPivot() {
    // Configure the Motor
    TalonFXConfiguration motorConfig = new TalonFXConfiguration();
    if (RobotBase.isReal()) {
      motorConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
    } else {
      motorConfig.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
    }
    motorConfig.Slot0.kP = PIVOT.kP;
    motorConfig.Slot0.kI = PIVOT.kI;
    motorConfig.Slot0.kD = PIVOT.kD;
    motorConfig.Slot0.kG = PIVOT.kGPositive;
    motorConfig.Slot0.GravityType = PIVOT.K_GRAVITY_TYPE_VALUE;
    motorConfig.MotionMagic.MotionMagicCruiseVelocity = PIVOT.kMotionMagicVelocity;
    motorConfig.MotionMagic.MotionMagicAcceleration = PIVOT.kMotionMagicAcceleration;
    motorConfig.MotorOutput.NeutralMode = m_neutralMode;
    //    motorConfig.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.RemoteCANcoder;
    motorConfig.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.FusedCANcoder;
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
      encoderConfig.MagnetSensor.SensorDirection = PIVOT.encoderDirection;
    }
    CtreUtils.configureCANCoder(m_pivotEncoder, encoderConfig);

    if (RobotBase.isSimulation()) m_pivotEncoder.setPosition(PIVOT_SETPOINT.STOWED.get());
    m_pivotMotor.setPosition(getCANcoderAngle());

    setName("EndEffectorPivot");
    SmartDashboard.putData(this);
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

  @Logged(name = "Desired Angle Degrees", importance = Logged.Importance.INFO)
  public double getDesiredRotationDegrees() {
    return m_desiredRotation.in(Degrees);
  }

  @Logged(name = "At Setpoint", importance = Logged.Importance.DEBUG)
  public boolean atSetpoint() {
    return m_desiredRotation.minus(getAngle()).abs(Degree) <= 1.0;
  }

  public void setPercentOutput(double speed) {
    m_pivotMotor.set(speed);
  }

  @Logged(name = "Motor Output", importance = Logged.Importance.INFO)
  public double getPercentOutput() {
    return m_pivotMotor.get();
  }

  @Logged(name = "Control Request", importance = Logged.Importance.DEBUG)
  public String getCurrentControlRequestString() {
    return m_pivotMotor.getAppliedControl().toString();
  }

  // Base unit from CANcoder is in Radians
  public Angle getCANcoderAngle() {
    m_canCoderAbsolutePositionSignal.refresh();
    return m_canCoderAbsolutePositionSignal.refresh().getValue();
  }

  @Logged(name = "CANcoder Angle Degrees", importance = Logged.Importance.INFO)
  public double getCANcoderAngleDegrees() {
    return getCANcoderAngle().in(Degrees);
  }

  public Angle getAngle() {
    return m_positionSignal.refresh().getValue();
  }

  @Logged(name = "Current Angle Degrees", importance = Logged.Importance.INFO)
  public double getAngleDegrees() {
    return getAngle().in(Degrees);
  }

  public AngularVelocity getVelocity() {
    return m_velocitySignal.refresh().getValue();
  }

  @Logged(name = "Velocity Degrees-s", importance = Logged.Importance.INFO)
  public double getVelocityDegrees() {
    return getVelocity().in(DegreesPerSecond);
  }

  public AngularAcceleration getAcceleration() {
    return m_accelSignal.refresh().getValue();
  }

  @Logged(name = "Acceleration Degrees-s^2", importance = Logged.Importance.INFO)
  public double getAccelerationDegrees() {
    return getAcceleration().in(DegreesPerSecondPerSecond);
  }

  public Voltage getMotorVoltage() {
    return m_voltageSignal.refresh().getValue();
  }

  public Current getSupplyCurrent() {
    return m_supplyCurrentSignal.refresh().getValue();
  }

  public Current getStatorCurrent() {
    return m_statorCurrentSignal.refresh().getValue();
  }

  public Current getTorqueCurrent() {
    return m_torqueCurrentSignal.refresh().getValue();
  }

  public void resetMotionMagicState() {
    m_desiredRotation = getAngle();
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

  public void setJoystickY(double m_joystickY) {
    m_joystickInput = m_joystickY;
  }

  public void teleopInit() {
    resetMotionMagicState();
  }
  
  public void testInit() {
    endEffectorPivotTab.getDoubleTopic("kP").publish().set(PIVOT.kP);
    endEffectorPivotTab.getDoubleTopic("kI").publish().set(PIVOT.kI);
    endEffectorPivotTab.getDoubleTopic("kD").publish().set(PIVOT.kD);
    endEffectorPivotTab
        .getDoubleTopic("MotionMagicCruiseVelocity")
        .publish()
        .set(PIVOT.kMotionMagicVelocity);
    endEffectorPivotTab
        .getDoubleTopic("MotionMagicAcceleration")
        .publish()
        .set(PIVOT.kMotionMagicAcceleration);
    endEffectorPivotTab
        .getDoubleTopic("Setpoint Degrees")
        .publish()
        .set(m_desiredRotation.in(Degrees));
    m_kP_subscriber = endEffectorPivotTab.getDoubleTopic("kP").subscribe(PIVOT.kP);
    m_kI_subscriber = endEffectorPivotTab.getDoubleTopic("kI").subscribe(PIVOT.kI);
    m_kD_subscriber = endEffectorPivotTab.getDoubleTopic("kD").subscribe(PIVOT.kD);
    m_velocitySubscriber =
        endEffectorPivotTab
            .getDoubleTopic("MotionMagicCruiseVelocity")
            .subscribe(PIVOT.kMotionMagicVelocity);
    m_accelerationSubscriber =
        endEffectorPivotTab
            .getDoubleTopic("MotionMagicAcceleration")
            .subscribe(PIVOT.kMotionMagicAcceleration);
    
    m_setpointSubscriber = endEffectorPivotTab.getDoubleTopic("Setpoint Degrees").subscribe(m_desiredRotation.in(Degrees));
  }

  public void testPeriodic() {
    Slot0Configs slot0Configs = new Slot0Configs();
    
    slot0Configs.kP = m_kP_subscriber.get(ELEVATOR.kP);
    slot0Configs.kI = m_kI_subscriber.get(ELEVATOR.kI);
    slot0Configs.kD = m_kD_subscriber.get(ELEVATOR.kD);

    m_pivotMotor.getConfigurator().apply(slot0Configs);

    MotionMagicConfigs motionMagicConfigs = new MotionMagicConfigs();

    motionMagicConfigs.MotionMagicCruiseVelocity =
        m_velocitySubscriber.get(ELEVATOR.motionMagicCruiseVelocity);
    motionMagicConfigs.MotionMagicAcceleration =
        m_accelerationSubscriber.get(ELEVATOR.motionMagicAcceleration);

    m_pivotMotor.getConfigurator().apply(motionMagicConfigs);
    
    double newSetpoint = m_setpointSubscriber.get(m_desiredRotation.in(Degrees));
    if (newSetpoint != m_desiredRotation.in(Degrees)) {
      setPosition(Degrees.of(newSetpoint));
      setControlMode(CONTROL_MODE.CLOSED_LOOP);
    }
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
  }

  @Override
  public void simulationPeriodic() {
    m_pivotMotorSimState.setSupplyVoltage(RobotController.getBatteryVoltage());
    m_pivotEncoderSimState.setSupplyVoltage(RobotController.getBatteryVoltage());
    m_endEffectorSim.setInputVoltage(m_pivotMotorSimState.getMotorVoltage());

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
