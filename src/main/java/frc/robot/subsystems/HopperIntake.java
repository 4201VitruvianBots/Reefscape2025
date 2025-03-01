package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.sim.TalonFXSimState;
import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.CAN;
import frc.robot.constants.HOPPERINTAKE;
import org.team4201.codex.utils.CtreUtils;

public class HopperIntake extends SubsystemBase {

  private final TalonFX m_hopperIntakeMotor = new TalonFX(CAN.hopperIntakeMotor);

  private final TalonFXSimState m_hopperIntakeMotorSimState = m_hopperIntakeMotor.getSimState();

  private final DCMotorSim m_hopperIntakeMotorSim =
      new DCMotorSim(
          LinearSystemId.createDCMotorSystem(
              HOPPERINTAKE.gearbox, HOPPERINTAKE.gearRatio, HOPPERINTAKE.kInertia),
          HOPPERINTAKE.gearbox);
  private final Servo m_hopperServo = new Servo(9);

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

  public void moveServo(double speed) {
    m_hopperServo.set(speed);
  }

  public void stopServo() {
    m_hopperServo.setAngle(90.0);
  }

  @Logged(name = "Motor Output", importance = Logged.Importance.INFO)
  public double getPercentOutput() {
    return m_hopperIntakeMotor.get();
  }

  public void teleopInit() {
    stopServo();
  }

  @Override
  public void simulationPeriodic() {
    m_hopperIntakeMotorSimState.setSupplyVoltage(RobotController.getBatteryVoltage());

    m_hopperIntakeMotorSim.setInputVoltage(m_hopperIntakeMotorSimState.getMotorVoltage());

    m_hopperIntakeMotorSim.update(0.02); // TODO update this later maybe?

    m_hopperIntakeMotorSimState.setRawRotorPosition(
        m_hopperIntakeMotorSim.getAngularPositionRotations() * HOPPERINTAKE.gearRatio);
    m_hopperIntakeMotorSimState.setRotorVelocity(
        m_hopperIntakeMotorSim.getAngularVelocityRPM() * HOPPERINTAKE.gearRatio / 60.0);
  }

  @Override
  public void periodic() {}
}
