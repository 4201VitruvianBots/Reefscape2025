package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.sim.TalonFXSimState;
import edu.wpi.first.wpilibj.CAN;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.HOPPERINTAKE;
import frc.robot.constants.V2CAN;
import frc.robot.utils.CtreUtils;

public class HopperIntake extends SubsystemBase {

  private final TalonFX m_hopperIntakeMotors[] = {new TalonFX(V2CAN.hopperIntakeMotor1), new TalonFX(V2CAN.hopperIntakeMotor2)};
  
  public HopperIntake() {
    TalonFXConfiguration config = new TalonFXConfiguration();
    config.Slot0.kP = HOPPERINTAKE.kP;
    config.Slot0.kI = HOPPERINTAKE.kI;
    config.Slot0.kD = HOPPERINTAKE.kD;
    config.Feedback.SensorToMechanismRatio = HOPPERINTAKE.gearRatio;
    config.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    config.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
    CtreUtils.configureTalonFx(m_hopperIntakeMotors[0], config);
    TalonFXConfiguration configBack = new TalonFXConfiguration();
    configBack.Slot0.kP = HOPPERINTAKE.kP;
    configBack.Slot0.kI = HOPPERINTAKE.kI;
    configBack.Slot0.kD = HOPPERINTAKE.kD;
    configBack.Feedback.SensorToMechanismRatio = HOPPERINTAKE.gearRatio;
    configBack.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    configBack.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
    CtreUtils.configureTalonFx(m_hopperIntakeMotors[1], configBack);
  }

  public void SetPercentOutput(double speed1, double speed2){
    m_hopperIntakeMotors[0].set(speed1);
    m_hopperIntakeMotors[1].set(speed2);
  }

  @Override
  public void periodic() {
  }
}
