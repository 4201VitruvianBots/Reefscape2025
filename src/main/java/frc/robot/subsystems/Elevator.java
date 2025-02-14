// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicVelocityVoltage;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.sim.TalonFXSimState;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.networktables.DoubleSubscriber;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.simulation.ElevatorSim;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.CAN;
import frc.robot.constants.ELEVATOR;
import frc.robot.constants.ROBOT.CONTROL_MODE;
import frc.robot.utils.CtreUtils;

public class Elevator extends SubsystemBase {

  /** Creates a new Elevator */
  private final TalonFX[] elevatorMotors = {
    new TalonFX(CAN.elevatorMotor1), new TalonFX(CAN.elevatorMotor2) // These are just placeholders
  };

  // Simulation classes help us simulate what's going on, including gravity.
  private final ElevatorSim m_elevatorSim =
      new ElevatorSim(
          ELEVATOR.gearbox,
          ELEVATOR.kElevatorGearing,
          ELEVATOR.kCarriageMassPounds,
          ELEVATOR.kElevatorDrumRadius,
          ELEVATOR.lowerLimitMeters,
          ELEVATOR.upperLimitMeters,
          true,
          0,
          0.0,
          0.0);
  private final StatusSignal<Angle> m_positionSignal = elevatorMotors[0].getPosition().clone();
  private final StatusSignal<Voltage> m_voltageSignal = elevatorMotors[0].getMotorVoltage().clone();
  private double m_desiredPositionMeters;
  // private boolean m_elevatorInitialized; I think this is for LEDs so have fun with that :)
  private double m_joystickInput;
  private CONTROL_MODE m_controlMode = CONTROL_MODE.OPEN_LOOP;
  private NeutralModeValue m_neutralMode = NeutralModeValue.Brake;
  private final MotionMagicVoltage m_request = new MotionMagicVoltage(0);
  private final MotionMagicVelocityVoltage m_requestVelocity = new MotionMagicVelocityVoltage(0);
  private final TalonFXSimState m_motorSimState;
  private DoubleSubscriber m_kP_subscriber,
      m_kI_subscriber,
      m_kD_subscriber,
      m_kASubscriber,
      m_kVSubscriber,
      m_velocitySubscriber,
      m_accelerationSubscriber,
      m_jerkSubscriber;
  private final NetworkTable elevatorTab =
      NetworkTableInstance.getDefault().getTable("Shuffleboard").getSubTable("Elevator");

  public Elevator() {
    TalonFXConfiguration config = new TalonFXConfiguration();
    config.Slot0.kP = ELEVATOR.kP;
    config.Slot0.kI = ELEVATOR.kI;
    config.Slot0.kD = ELEVATOR.kD;
    // config.Slot0.kA = ELEVATOR.kA;
    // config.Slot0.kV = ELEVATOR.kV;    
    config.Feedback.SensorToMechanismRatio = ELEVATOR.gearRatio;
    config.MotionMagic.MotionMagicCruiseVelocity = ELEVATOR.motionMagicCruiseVelocity;
    config.MotionMagic.MotionMagicAcceleration = ELEVATOR.motionMagicAcceleration;
    config.MotionMagic.MotionMagicJerk = ELEVATOR.motionMagicJerk;
    config.CurrentLimits.StatorCurrentLimit = 40;
    CtreUtils.configureTalonFx(elevatorMotors[0], config);
    CtreUtils.configureTalonFx(elevatorMotors[1], config);
    m_motorSimState = elevatorMotors[0].getSimState();
    elevatorMotors[1].setControl(new Follower(elevatorMotors[0].getDeviceID(), true));
    SmartDashboard.putData(this);
  }

  public void holdElevator() {
    setDesiredPosition(getHeightMeters());
  }

  public void setPercentOutput(double output) {
    elevatorMotors[0].set(output);
  }

  public double getPercentOutputMotor() {
    return elevatorMotors[0].get();
  }

  public void setDesiredPosition(double desiredPosition) {
    m_desiredPositionMeters = desiredPosition;
  }

  public void setDesiredAcceleration(double desiredAccel) {
    m_requestVelocity.Acceleration = desiredAccel;
  }

  public void setJoystickInput(double joystickInput) {
    m_joystickInput = joystickInput;
  }

  public void setControlMode(CONTROL_MODE controlMode) {
    m_controlMode = controlMode;
  }

  public CONTROL_MODE getControlMode() {
    return m_controlMode;
  }

  public Double getMotorRotations() {
    m_positionSignal.refresh();
    return m_positionSignal.getValueAsDouble();
  }

  public CONTROL_MODE getClosedLoopControlMode() {
    return m_controlMode;
  }

  public Double getMotorVoltage() {
    m_voltageSignal.refresh();
    return m_voltageSignal.getValueAsDouble();
  }

  public void setJoystickY(double m_joystickY) {
    m_joystickInput = m_joystickY;
  }

  public double getHeightMeters() {
    return getMotorRotations() * ELEVATOR.sprocketRotationsToMeters;
  }

  // Sets the control state of the elevator
  public void setClosedLoopControlMode(CONTROL_MODE mode) {
    m_controlMode = mode;
  }

  public boolean isClosedLoopControl() {
    return getClosedLoopControlMode() == CONTROL_MODE.CLOSED_LOOP;
  }

  public void setElevatorNeutralMode(NeutralModeValue mode) {
    if (mode == m_neutralMode) return;
    m_neutralMode = mode;
    elevatorMotors[0].setNeutralMode(mode);
  }

  public NeutralModeValue getNeutralMode() {
    return m_neutralMode;
  }


  public void testInit(){
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
    elevatorTab.getDoubleTopic("MotionMagicJerk").publish().set(ELEVATOR.motionMagicJerk);
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
    m_jerkSubscriber =
        elevatorTab.getDoubleTopic("MotionMagicJerk").subscribe(ELEVATOR.motionMagicJerk);
  }

  public void testPeriodic() {
    Slot0Configs slot0Configs = new Slot0Configs();
    MotionMagicConfigs motionMagicConfigs = new MotionMagicConfigs();
    slot0Configs.kP = m_kP_subscriber.get(ELEVATOR.kP);
    slot0Configs.kI = m_kI_subscriber.get(ELEVATOR.kI);
    slot0Configs.kD = m_kD_subscriber.get(ELEVATOR.kD);
    slot0Configs.kA = m_kASubscriber.get(ELEVATOR.kA);
    slot0Configs.kV = m_kVSubscriber.get(ELEVATOR.kV);
    // Who on earth knows if this part works, I was guessing that it would.
    motionMagicConfigs.MotionMagicCruiseVelocity =
        m_velocitySubscriber.get(ELEVATOR.motionMagicCruiseVelocity);
    motionMagicConfigs.MotionMagicAcceleration =
        m_accelerationSubscriber.get(ELEVATOR.motionMagicAcceleration);
    motionMagicConfigs.MotionMagicJerk = m_jerkSubscriber.get(ELEVATOR.motionMagicJerk);
  }

  public void teleopInit() {
    holdElevator();
    setDesiredPosition(getHeightMeters());
  }

  @Override
  public void simulationPeriodic() {
    m_motorSimState.setSupplyVoltage(RobotController.getBatteryVoltage());

    m_elevatorSim.update(0.020);

    m_elevatorSim.setInputVoltage(MathUtil.clamp(m_motorSimState.getMotorVoltage(), -12, 12));
    m_motorSimState.setSupplyVoltage(RobotController.getBatteryVoltage());

    m_motorSimState.setRawRotorPosition(
        m_elevatorSim.getPositionMeters()
            * ELEVATOR.gearRatio
            / ELEVATOR.sprocketRotationsToMeters);
    m_motorSimState.setRotorVelocity(
        m_elevatorSim.getVelocityMetersPerSecond()
            * ELEVATOR.gearRatio
            / ELEVATOR.sprocketRotationsToMeters);
  }

  private void updateSmartDashboard() {
    SmartDashboard.putNumber("Elevator Height", getHeightMeters());
    SmartDashboard.putNumber("Elevator Desired Height", m_desiredPositionMeters);
    SmartDashboard.putNumber("Motor Voltage", getMotorVoltage());
    SmartDashboard.putNumber("Motor Rotations", getMotorRotations());
    SmartDashboard.putNumber("Joystick Input", m_joystickInput);
    SmartDashboard.putBoolean("isClosedLoop", isClosedLoopControl());
    SmartDashboard.putNumber("Elevator Velocity", m_requestVelocity.Velocity);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    switch (m_controlMode) {
      case CLOSED_LOOP_NET:
        // m_requestVelocity.Acceleration = 100; TODO: figure out where to put this.
        elevatorMotors[0].setControl(m_requestVelocity.withVelocity(80));
        break;
      case CLOSED_LOOP:
        elevatorMotors[0].setControl(m_request.withPosition(m_desiredPositionMeters));
        break;
      case OPEN_LOOP:
      default:
        double percentOutput = m_joystickInput * ELEVATOR.kPercentOutputMultiplier + ELEVATOR.offset; 
        setPercentOutput(percentOutput); 
        break;
    }
    updateSmartDashboard();
  }
}
