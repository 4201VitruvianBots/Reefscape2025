// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import static edu.wpi.first.units.Units.*; // I'll use this later don't worrrryyyyy

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicTorqueCurrentFOC;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.sim.TalonFXSimState;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.simulation.ElevatorSim;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.ELEVATOR;
import frc.robot.constants.ROBOT.CONTROL_MODE;
import frc.robot.constants.V2CAN;
import frc.robot.utils.CtreUtils;

public class Elevator extends SubsystemBase {

  /** Creates a new Elevator */
  private final TalonFX[] elevatorMotors = {
    new TalonFX(V2CAN.elevatorMotor1),
    new TalonFX(V2CAN.elevatorMotor2) // These are just placeholders
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
          false,
          0,
          0.0,
          0.0);
  private final StatusSignal<Angle> m_positionSignal = elevatorMotors[0].getPosition().clone();
  private final StatusSignal<Voltage> m_voltageSignal = elevatorMotors[0].getMotorVoltage().clone();
  private double m_desiredPositionMeters;
  private boolean m_elevatorInitialized;
  private double m_joystickInput = 0.0;
  private CONTROL_MODE m_controlMode = CONTROL_MODE.OPEN_LOOP;
  private NeutralModeValue m_neutralMode = NeutralModeValue.Brake;
  private final MotionMagicTorqueCurrentFOC m_request = new MotionMagicTorqueCurrentFOC(0);
  private final TalonFXSimState m_motorSimState;

  public Elevator() {
    TalonFXConfiguration configElevator = new TalonFXConfiguration();
    configElevator.Slot0.kP = ELEVATOR.kP;
    configElevator.Slot0.kI = ELEVATOR.kI;
    configElevator.Slot0.kD = ELEVATOR.kD;
    configElevator.Slot0.kA = ELEVATOR.kA;
    configElevator.Slot0.kV = ELEVATOR.kV;
    CtreUtils.configureTalonFx(elevatorMotors[0], configElevator);
    CtreUtils.configureTalonFx(elevatorMotors[1], configElevator);

    configElevator.MotionMagic.MotionMagicCruiseVelocity = 100;
    configElevator.MotionMagic.MotionMagicAcceleration = 200;
    m_motorSimState = elevatorMotors[0].getSimState();
    elevatorMotors[1].setControl(new Follower(elevatorMotors[0].getDeviceID(), false));
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

  public void setJoystickInput(double joystickInput) {
    m_joystickInput = joystickInput;
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

  public void setClimberNeutralMode(NeutralModeValue mode) {
    if (mode == m_neutralMode) return;
    m_neutralMode = mode;
    elevatorMotors[0].setNeutralMode(mode);
  }

  public void teleopInit() {
    holdElevator();
    setDesiredPosition(getHeightMeters());
  }

  


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
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    switch (m_controlMode) {
      case CLOSED_LOOP:
        elevatorMotors[0].setControl(m_request.withPosition(m_desiredPositionMeters));
        break;
      case OPEN_LOOP:
      default:
        double percentOutput = m_joystickInput * ELEVATOR.kPercentOutputMultiplier;
        setPercentOutput(percentOutput); 
        break;
    }
    updateSmartDashboard();
  }
}
