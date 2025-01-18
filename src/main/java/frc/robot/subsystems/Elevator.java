// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.ELEVATOR;
import frc.robot.utils.CtreUtils;

public class Elevator extends SubsystemBase {

  /** Creates a new Elevator */
  private final TalonFX[] elevatorMotors = {
    new TalonFX(0), new TalonFX(1) // These are just placeholders
  };

  private final StatusSignal<Angle> m_positionSignal =
      elevatorMotors[0].getPosition().clone();
  private final StatusSignal<Angle> m_positionSignal2 =
      elevatorMotors[1].getPosition().clone();

  private double m_desiredPositionMeters;
  private boolean m_elevatorInitialized;

  public Elevator() {
    TalonFXConfiguration configElevator = new TalonFXConfiguration();
    configElevator.Slot0.kP = ELEVATOR.kPBottom;
    configElevator.Slot0.kI = ELEVATOR.kIBottom;
    configElevator.Slot0.kD = ELEVATOR.kDBottom;
    CtreUtils.configureTalonFx(elevatorMotors[0], configElevator);
    CtreUtils.configureTalonFx(elevatorMotors[1], configElevator);
  }

  private Angle getMotorRotations(){
    m_positionSignal.refresh();
    return m_positionSignal.getValue();
  }

  // private Angle getPositionMeters() {
  //   return getMotorRotations() * ELEVATOR.sprocketRotationsToMeters;3
  // }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}
