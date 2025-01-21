// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicTorqueCurrentFOC;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.CAN;
import frc.robot.constants.ELEVATOR;
import frc.robot.constants.ROBOT;
import frc.robot.constants.ROBOT.CONTROL_MODE;
import frc.robot.utils.CtreUtils;

public class Elevator extends SubsystemBase {

  /** Creates a new Elevator */
  private final TalonFX[] elevatorMotors = {
    new TalonFX(CAN.elevatorMotor1), new TalonFX(CAN.elevatorMotor2) // These are just placeholders
  };

  private final StatusSignal<Angle> m_positionSignal =
      elevatorMotors[0].getPosition().clone();
  private final StatusSignal<Angle> m_positionSignal2 =
      elevatorMotors[1].getPosition().clone();

  private double m_desiredPositionMeters;
  private boolean m_elevatorInitialized;
  private double m_joystickInput = 0.0;
  private ROBOT.CONTROL_MODE m_controlMode = CONTROL_MODE.OPEN_LOOP;
  private NeutralModeValue m_neutralMode = NeutralModeValue.Brake;
  private final MotionMagicTorqueCurrentFOC m_request = new MotionMagicTorqueCurrentFOC(0);

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
    elevatorMotors[1].setControl(new Follower(elevatorMotors[0].getDeviceID(), false));
  }

  private Angle getMotorRotations() {
    m_positionSignal.refresh();
    return m_positionSignal.getValue();
  }
  
  public void setPercentOutput(double output) {
    elevatorMotors[0].set(output);
    elevatorMotors[1].set(output);
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

  

  // private Angle getPositionMeters() {
  //   return getMotorRotations() * ELEVATOR.sprocketRotationsToMeters;
  // }

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
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}
