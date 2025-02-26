// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicVelocityVoltage;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.sim.TalonFXSimState;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.epilogue.NotLogged;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.networktables.DoubleSubscriber;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularAcceleration;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.LinearAcceleration;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.simulation.ElevatorSim;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.CAN;
import frc.robot.constants.ELEVATOR;
import frc.robot.constants.ELEVATOR.ELEVATOR_ACCEL_SETPOINT;
import frc.robot.constants.ELEVATOR.ELEVATOR_SETPOINT;
import frc.robot.constants.ROBOT.CONTROL_MODE;
import frc.robot.utils.CtreUtils;

@Logged
public class Elevator extends SubsystemBase {
  /** Creates a new Elevator */
  @NotLogged private final TalonFX[] elevatorMotors = {
    new TalonFX(CAN.elevatorMotor1), new TalonFX(CAN.elevatorMotor2)
  };

  // Simulation classes help us simulate what's going on, including gravity.
  private final ElevatorSim m_elevatorSim =
      new ElevatorSim(
          ELEVATOR.gearbox,
          ELEVATOR.gearRatio,
          ELEVATOR.kCarriageMassPounds,
          ELEVATOR.kElevatorDrumDiameter / 2,
          ELEVATOR.lowerLimitMeters,
          ELEVATOR.upperLimitMeters,
          true,
          0,
          0.0,
          0.0);

  @NotLogged private final StatusSignal<Angle> m_positionSignal = elevatorMotors[0].getPosition().clone();
  @NotLogged private final StatusSignal<Voltage> m_voltageSignal = elevatorMotors[0].getMotorVoltage().clone();
  @NotLogged private final StatusSignal<Current> m_currentSignal =
      elevatorMotors[0].getTorqueCurrent().clone();
  @NotLogged private final StatusSignal<AngularVelocity> m_velocitySignal =
      elevatorMotors[0].getVelocity().clone();
  @NotLogged private final StatusSignal<AngularAcceleration> m_accelSignal =
      elevatorMotors[0].getAcceleration().clone();

  private Distance m_desiredPosition;
  private double m_joystickInput;
  private CONTROL_MODE m_controlMode = CONTROL_MODE.OPEN_LOOP;
  private NeutralModeValue m_neutralMode = NeutralModeValue.Brake;

  @NotLogged private final MotionMagicVoltage m_request = new MotionMagicVoltage(0);
  @NotLogged private final MotionMagicVelocityVoltage m_requestVelocity = new MotionMagicVelocityVoltage(0);

  @NotLogged private final TalonFXSimState m_motorSimState;

  @NotLogged
  private DoubleSubscriber m_kP_subscriber,
      m_kI_subscriber,
      m_kD_subscriber,
      m_kASubscriber,
      m_kVSubscriber,
      m_velocitySubscriber,
      m_accelerationSubscriber;
  private final NetworkTable elevatorTab =
      NetworkTableInstance.getDefault().getTable("Shuffleboard").getSubTable("Elevator");

  public Elevator() {
    TalonFXConfiguration config = new TalonFXConfiguration();
    config.Slot0.kG = ELEVATOR.kG;
    config.Slot0.kS = ELEVATOR.kS;
    config.Slot0.kV = ELEVATOR.kV;
    config.Slot0.kA = ELEVATOR.kA;
    config.Slot0.kP = ELEVATOR.kP;
    config.Slot0.kI = ELEVATOR.kI;
    config.Slot0.kD = ELEVATOR.kD;

    config.Feedback.SensorToMechanismRatio = ELEVATOR.gearRatio;
    config.MotionMagic.MotionMagicCruiseVelocity = ELEVATOR.motionMagicCruiseVelocity;
    config.MotionMagic.MotionMagicAcceleration = ELEVATOR.motionMagicAcceleration;
    if (!RobotBase.isSimulation()) config.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
    config.MotionMagic.MotionMagicJerk = ELEVATOR.motionMagicJerk;
    config.CurrentLimits.StatorCurrentLimit = 40;
    config.MotorOutput.PeakReverseDutyCycle = ELEVATOR.peakReverseOutput;
    config.MotorOutput.PeakForwardDutyCycle = ELEVATOR.peakForwardOutput;
    config.MotorOutput.NeutralMode = m_neutralMode;

    CtreUtils.configureTalonFx(elevatorMotors[0], config);
    CtreUtils.configureTalonFx(elevatorMotors[1], config);

    m_motorSimState = elevatorMotors[0].getSimState();

    elevatorMotors[0].setPosition(Rotations.of(0));
    elevatorMotors[1].setControl(new Follower(elevatorMotors[0].getDeviceID(), true));

    SmartDashboard.putData(this);
  }

  public void holdElevator() {
    setDesiredPosition(getHeight());
  }

  public void setPercentOutput(double output) {
    elevatorMotors[0].set(output);
  }

  public double getPercentOutput() {
    return elevatorMotors[0].get();
  }

  public void setDesiredPosition(Distance desiredPosition) {
    m_desiredPosition = desiredPosition;
  }

  public void setDesiredAcceleration(AngularAcceleration desiredAccel) {
    m_requestVelocity.Acceleration = desiredAccel.in(RotationsPerSecondPerSecond);
  }

  public void setJoystickInput(double joystickInput) {
    m_joystickInput = joystickInput;
  }

  public void setControlMode(CONTROL_MODE controlMode) {
    m_controlMode = controlMode;
  }

  public Current getStatorCurrent() {
    m_voltageSignal.refresh();
    return elevatorMotors[0].getStatorCurrent().getValue();
  }

  public CONTROL_MODE getControlMode() {
    return m_controlMode;
  }

  public Angle getRotations() {
    m_positionSignal.refresh();
    return m_positionSignal.getValue();
  }

  public Current getCurrent() {
    m_currentSignal.refresh();
    return m_currentSignal.getValue();
  }

  public LinearVelocity getVelocity() {
    m_velocitySignal.refresh();
    return MetersPerSecond.of(m_velocitySignal.getValue().in(RotationsPerSecond) * ELEVATOR.drumRotationsToMeters);
  }

  public LinearAcceleration getAcceleration() {
    m_accelSignal.refresh();
    return MetersPerSecondPerSecond.of(m_accelSignal.getValue().in(RotationsPerSecondPerSecond)
        * ELEVATOR.drumRotationsToMeters);
  }

  public double getMotorVoltage() {
    m_voltageSignal.refresh();
    return m_voltageSignal.getValueAsDouble();
  }

  public void setJoystickY(double m_joystickY) {
    m_joystickInput = m_joystickY;
  }

  public Distance getHeight() {
    return Meters.of(getRotations().in(Rotations) * ELEVATOR.drumRotationsToMeters);
  }

  public boolean isClosedLoopControl() {
    return getControlMode() == CONTROL_MODE.CLOSED_LOOP;
  }

  public void setNeutralMode(NeutralModeValue mode) {
    if (mode == m_neutralMode) return;
    m_neutralMode = mode;
    elevatorMotors[0].setNeutralMode(mode);
  }

  public NeutralModeValue getNeutralMode() {
    return m_neutralMode;
  }

  public Distance getDesiredHeight() {
    return m_desiredPosition;
  }

  // Elevator is within 1 inch of its set]point
  public boolean atSetpoint() {
    return m_desiredPosition.minus(getHeight()).abs(Inches) <= 1; // RIP the 254 reference
  }

  public void testInit() {
    elevatorTab.getDoubleTopic("kP").publish().set(ELEVATOR.kP);
    elevatorTab.getDoubleTopic("kI").publish().set(ELEVATOR.kI);
    elevatorTab.getDoubleTopic("kD").publish().set(ELEVATOR.kD);
    elevatorTab.getDoubleTopic("kA").publish().set(ELEVATOR.kA);
    elevatorTab.getDoubleTopic("kV").publish().set(ELEVATOR.kV);
    elevatorTab
        .getDoubleTopic("MotionMagicCruiseVelocity")
        .publish()
        .set(ELEVATOR.motionMagicCruiseVelocity);
    elevatorTab
        .getDoubleTopic("MotionMagicAcceleration")
        .publish()
        .set(ELEVATOR.motionMagicAcceleration);
    // elevatorTab.getDoubleTopic("MotionMagicJerk").publish().set(ELEVATOR.motionMagicJerk);
    m_kP_subscriber = elevatorTab.getDoubleTopic("kP").subscribe(ELEVATOR.kP);
    m_kI_subscriber = elevatorTab.getDoubleTopic("kI").subscribe(ELEVATOR.kI);
    m_kD_subscriber = elevatorTab.getDoubleTopic("kD").subscribe(ELEVATOR.kD);
    m_kASubscriber = elevatorTab.getDoubleTopic("kA").subscribe(ELEVATOR.kA);
    m_kVSubscriber = elevatorTab.getDoubleTopic("kV").subscribe(ELEVATOR.kV);
    m_velocitySubscriber =
        elevatorTab
            .getDoubleTopic("MotionMagicCruiseVelocity")
            .subscribe(ELEVATOR.motionMagicCruiseVelocity);
    m_accelerationSubscriber =
        elevatorTab
            .getDoubleTopic("MotionMagicAcceleration")
            .subscribe(ELEVATOR.motionMagicAcceleration);
    // m_jerkSubscriber =
    //     elevatorTab.getDoubleTopic("MotionMagicJerk").subscribe(ELEVATOR.motionMagicJerk);
  }

  public void testPeriodic() {
    Slot0Configs slot0Configs = new Slot0Configs();

    slot0Configs.kP = m_kP_subscriber.get(ELEVATOR.kP);
    slot0Configs.kI = m_kI_subscriber.get(ELEVATOR.kI);
    slot0Configs.kD = m_kD_subscriber.get(ELEVATOR.kD);
    slot0Configs.kA = m_kASubscriber.get(ELEVATOR.kA);
    slot0Configs.kV = m_kVSubscriber.get(ELEVATOR.kV);

    elevatorMotors[0].getConfigurator().apply(slot0Configs);
    elevatorMotors[1].getConfigurator().apply(slot0Configs);

    MotionMagicConfigs motionMagicConfigs = new MotionMagicConfigs();

    // Who on earth knows if this part works, I was guessing that it would.
    motionMagicConfigs.MotionMagicCruiseVelocity =
        m_velocitySubscriber.get(ELEVATOR.motionMagicCruiseVelocity);
    motionMagicConfigs.MotionMagicAcceleration =
        m_accelerationSubscriber.get(ELEVATOR.motionMagicAcceleration);
    // motionMagicConfigs.MotionMagicJerk = m_jerkSubscriber.get(ELEVATOR.motionMagicJerk);

    elevatorMotors[0].getConfigurator().apply(motionMagicConfigs);
    elevatorMotors[1].getConfigurator().apply(motionMagicConfigs);
  }

  public void teleopInit() {
    holdElevator();
  }

  @Override
  public void simulationPeriodic() {
    m_motorSimState.setSupplyVoltage(RobotController.getBatteryVoltage());

    m_elevatorSim.update(0.020);

    m_elevatorSim.setInputVoltage(MathUtil.clamp(m_motorSimState.getMotorVoltage(), -12, 12));
    m_motorSimState.setSupplyVoltage(RobotController.getBatteryVoltage());

    m_motorSimState.setRawRotorPosition(
        m_elevatorSim.getPositionMeters() * ELEVATOR.gearRatio / ELEVATOR.drumRotationsToMeters);
    m_motorSimState.setRotorVelocity(
        m_elevatorSim.getVelocityMetersPerSecond()
            * ELEVATOR.gearRatio
            / ELEVATOR.drumRotationsToMeters);
  }

  private void updateSmartDashboard() {
    // SmartDashboard.putNumber("Elevator/Height Inches", getHeight().in(Inches));
    // SmartDashboard.putNumber("Elevator/Elevator Desired Height", m_desiredPositionMeters);
    // SmartDashboard.putNumber("Elevator/Elevator Velocity Mps", getVelocity());
    // SmartDashboard.putNumber("Elevator/Motor Voltage", getMotorVoltage());
    // SmartDashboard.putNumber("Elevator/Elevator Torque Current", getCurrent());
    // SmartDashboard.putNumber("Elevator/Acceleration", getAccelMps());
    // SmartDashboard.putBoolean("Elevator/Is Closed Loop", isClosedLoopControl());
    // SmartDashboard.putNumber("Elevator/Motor Rotations", getMotorRotations());
    // SmartDashboard.putNumber("Elevator/Joystick Input", m_joystickInput);
    // SmartDashboard.putNumber("Elevator/Elevator Velocity Setpoint", m_requestVelocity.Velocity);
    // SmartDashboard.putString("Elevator/Neutral Mode", m_neutralMode.toString());
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    switch (m_controlMode) {
      case CLOSED_LOOP_NET:
        elevatorMotors[0].setControl(m_requestVelocity.withVelocity(ELEVATOR_ACCEL_SETPOINT.NETSCORE.getVelocity()));
        break;
      case CLOSED_LOOP:
        if (atSetpoint()
            && m_desiredPosition.minus(ELEVATOR_SETPOINT.START_POSITION.getSetpoint()).abs(Inches) <= 1) {
          elevatorMotors[0].set(0); // Don't move the elevator if already at stowed
        } else {
          elevatorMotors[0].setControl(
              m_request.withPosition(m_desiredPosition.in(Meters) / ELEVATOR.drumRotationsToMeters));
        }
        break;
      case OPEN_LOOP:
      default:
        double percentOutput;
        if (m_joystickInput < 0) {
          percentOutput =
              m_joystickInput * ELEVATOR.kLimitedPercentOutputMultiplier + ELEVATOR.offset;
        } else {
          percentOutput = m_joystickInput * ELEVATOR.kPercentOutputMultiplier + ELEVATOR.offset;
        }
        setPercentOutput(percentOutput);
        break;
    }
    updateSmartDashboard();
  }
}
