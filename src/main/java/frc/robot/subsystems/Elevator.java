// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.utils.CtreUtils;

public class Elevator extends SubsystemBase {
  /** Creates a new Elevator */
  private final TalonFX[] elevatormotors = {
    new TalonFX(0), new TalonFX(1) // These are just placeholders
  };

  private double m_desiredPositionMeters;
  private boolean m_elevatorInitialized;

  public Elevator() {
    TalonFXConfiguration configElevator = new TalonFXConfiguration();
    configElevator.Slot0.kP = frc.robot.constants.ELEVATOR.kPBottom;
    configElevator.Slot0.kI = frc.robot.constants.ELEVATOR.kIBottom;
    configElevator.Slot0.kD = frc.robot.constants.ELEVATOR.kDBottom;
    CtreUtils.configureTalonFx(elevatormotors[0], configElevator);
    CtreUtils.configureTalonFx(elevatormotors[1], configElevator);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}
