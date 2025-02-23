package frc.robot.subsystems;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.sim.TalonFXSimState;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.CAN;
import frc.robot.constants.HOPPERINTAKE;
import frc.robot.utils.CtreUtils;

public class HopperIntake extends SubsystemBase {

  private final TalonFX m_hopperIntakeMotor = new TalonFX(CAN.hopperIntakeMotor);
  private final StatusSignal<AngularVelocity> m_velocitySignal =
      m_hopperIntakeMotor.getVelocity().clone();
  private final StatusSignal<Voltage> m_voltageSignal =
      m_hopperIntakeMotor.getMotorVoltage().clone();
  private final StatusSignal<Current> m_currentSignal =
      m_hopperIntakeMotor.getTorqueCurrent().clone();
  private final TalonFXSimState m_hopperIntakeMotorSimState = m_hopperIntakeMotor.getSimState();
  private final DCMotorSim m_hopperIntakeMotorSim =
      new DCMotorSim(
          LinearSystemId.createDCMotorSystem(
              HOPPERINTAKE.gearbox, HOPPERINTAKE.gearRatio, HOPPERINTAKE.kInertia),
          HOPPERINTAKE.gearbox);

  public HopperIntake() {
    TalonFXConfiguration config = new TalonFXConfiguration();
    config.Slot0.kP = HOPPERINTAKE.kP;
    config.Slot0.kI = HOPPERINTAKE.kI;
    config.Slot0.kD = HOPPERINTAKE.kD;
    config.Feedback.SensorToMechanismRatio = HOPPERINTAKE.gearRatio;
    config.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    config.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
    config.MotorOutput.PeakReverseDutyCycle = HOPPERINTAKE.peakReverseOutput;
    config.MotorOutput.PeakForwardDutyCycle = HOPPERINTAKE.peakForwardOutput;
    CtreUtils.configureTalonFx(m_hopperIntakeMotor, config);
  }

  public void setPercentOutput(double speed) {
    m_hopperIntakeMotor.set(speed);
  }

  public void updateSmartDashboard() {
    SmartDashboard.putNumber("Hopper Intake/Motor Velocity", m_velocitySignal.getValueAsDouble());
    SmartDashboard.putNumber(
        "Hopper Intake/Motor Output", m_voltageSignal.getValueAsDouble() / 12.0);
    SmartDashboard.putNumber("Hopper Intake/Motor Current", m_currentSignal.getValueAsDouble());
  }

  @Override
  public void simulationPeriodic() {
    m_hopperIntakeMotorSimState.setSupplyVoltage(RobotController.getBatteryVoltage());

    m_hopperIntakeMotorSim.setInputVoltage(
        MathUtil.clamp(m_hopperIntakeMotorSimState.getMotorVoltage(), -12, 12));

    m_hopperIntakeMotorSim.update(0.02); // TODO update this later maybe?

    m_hopperIntakeMotorSimState.setRawRotorPosition(
        m_hopperIntakeMotorSim.getAngularPositionRotations() * HOPPERINTAKE.gearRatio);
    m_hopperIntakeMotorSimState.setRotorVelocity(
        m_hopperIntakeMotorSim.getAngularVelocityRPM() * HOPPERINTAKE.gearRatio / 60.0);
  }

  @Override
  public void periodic() {
    updateSmartDashboard();
  }
}
