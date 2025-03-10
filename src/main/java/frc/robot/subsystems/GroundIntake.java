package frc.robot.subsystems;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.sim.TalonFXSimState;
import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.CAN;
import frc.robot.constants.GROUND;
import frc.robot.constants.GROUND.INTAKE;
import org.team4201.codex.utils.CtreUtils;

public class GroundIntake extends SubsystemBase {
  private final TalonFX m_groundIntakeMotor = new TalonFX(CAN.groundRollerMotor);

  private final StatusSignal<AngularVelocity> m_velocitySignal =
      m_groundIntakeMotor.getVelocity().clone();
  private final StatusSignal<Voltage> m_voltageSignal =
      m_groundIntakeMotor.getMotorVoltage().clone();
  private final StatusSignal<Current> m_supplyCurrentSignal =
      m_groundIntakeMotor.getSupplyCurrent().clone();
  private final StatusSignal<Current> m_statorCurrentSignal =
      m_groundIntakeMotor.getStatorCurrent().clone();
  private final StatusSignal<Current> m_torqueCurrentSignal =
      m_groundIntakeMotor.getTorqueCurrent().clone();

  private final DCMotorSim m_groundIntakeMotorSim =
      new DCMotorSim(
          LinearSystemId.createDCMotorSystem(
              GROUND.INTAKE.gearbox, GROUND.INTAKE.kInertia, GROUND.INTAKE.gearRatio),
          GROUND.INTAKE.gearbox);
  private final TalonFXSimState m_groundIntakeMotorSimState = m_groundIntakeMotor.getSimState();

  public GroundIntake() {
    TalonFXConfiguration config = new TalonFXConfiguration();
    config.Slot0.kP = GROUND.INTAKE.kP;
    config.Slot0.kI = GROUND.INTAKE.kI;
    config.Slot0.kD = GROUND.INTAKE.kD;
    config.Feedback.SensorToMechanismRatio = GROUND.INTAKE.gearRatio;
    config.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    config.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
    CtreUtils.configureTalonFx(m_groundIntakeMotor, config);
  }

  public void setPercentOutput(double speed) {
    m_groundIntakeMotor.set(speed);
  }

  @Logged(name = "Motor Output", importance = Logged.Importance.INFO)
  public double getPercentOutput() {
    return m_groundIntakeMotor.get();
  }

  public AngularVelocity getVelocity() {
    return m_velocitySignal.refresh().getValue();
  }

  public Voltage getMotorVoltage() {
    return m_voltageSignal.refresh().getValue();
  }

  public Current getSupplyCurrent() {
    return m_supplyCurrentSignal.refresh().getValue();
  }

  public Current getStatorCurrent() {
    return m_statorCurrentSignal.refresh().getValue();
  }

  public Current getTorqueCurrent() {
    return m_torqueCurrentSignal.refresh().getValue();
  }

  @Override
  public void periodic() {}

  @Override
  public void simulationPeriodic() {
    m_groundIntakeMotorSimState.setSupplyVoltage(RobotController.getBatteryVoltage());

    m_groundIntakeMotorSim.setInputVoltage(m_groundIntakeMotorSimState.getMotorVoltage());

    m_groundIntakeMotorSim.update(0.02); // TODO update this later maybe?

    m_groundIntakeMotorSimState.setRawRotorPosition(
        m_groundIntakeMotorSim.getAngularPositionRotations() * GROUND.INTAKE.gearRatio);
    m_groundIntakeMotorSimState.setRotorVelocity(
        m_groundIntakeMotorSim.getAngularVelocityRPM() * GROUND.INTAKE.gearRatio / 60.0);
  }
}
