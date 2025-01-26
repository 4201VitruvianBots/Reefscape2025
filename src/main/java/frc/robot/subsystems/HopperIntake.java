package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.V2CAN;

public class HopperIntake extends SubsystemBase {

  private final TalonFX m_intakeMotors[] = {new TalonFX(V2CAN.hopperIntakeMotor1), new TalonFX(V2CAN.hopperIntakeMotor2)};

  public HopperIntake() {}

  @Override
  public void periodic() {
  }
}
