package frc.robot.subsystems;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.sim.TalonFXSimState;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.wpilibj.CAN;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.HOPPERINTAKE;
import frc.robot.constants.V2CAN;
import frc.robot.utils.CtreUtils;
import static edu.wpi.first.units.Units.*;

public class HopperIntake extends SubsystemBase {

  private final TalonFX m_hopperIntakeMotor = new TalonFX(V2CAN.hopperIntakeMotor);
   private final StatusSignal<AngularVelocity> m_velocitySignal1 = m_hopperIntakeMotor.getVelocity().clone();
  private final StatusSignal<Voltage> m_voltageSignal1 = m_hopperIntakeMotor.getMotorVoltage().clone();
  private final StatusSignal<Current> m_currentSignal1 = m_hopperIntakeMotor.getTorqueCurrent().clone();
  
  public HopperIntake() {
    TalonFXConfiguration config = new TalonFXConfiguration();
    config.Slot0.kP = HOPPERINTAKE.kP;
    config.Slot0.kI = HOPPERINTAKE.kI;
    config.Slot0.kD = HOPPERINTAKE.kD;
    config.Feedback.SensorToMechanismRatio = HOPPERINTAKE.gearRatio;
    config.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    config.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
    CtreUtils.configureTalonFx(m_hopperIntakeMotor, config);
  }

  public void setPercentOutput(double speed){
    m_hopperIntakeMotor.set(speed);
  }

  public void updateLogger(){
    SmartDashboard.putNumber("Hopper Intake/Motor", m_velocitySignal1.getValueAsDouble());
    SmartDashboard.putNumber("Hopper Intake/Motor1 Output", m_voltageSignal1.getValueAsDouble() / 12.0);
    SmartDashboard.putNumber("Hopper Intake/Motor1 Current", m_currentSignal1.getValueAsDouble());
  }

  @Override
  public void periodic() {
  }
}
