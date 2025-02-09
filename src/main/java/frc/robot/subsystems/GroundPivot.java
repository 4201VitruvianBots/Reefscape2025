package frc.robot.subsystems;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicTorqueCurrentFOC;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;
import com.ctre.phoenix6.sim.CANcoderSimState;
import com.ctre.phoenix6.sim.TalonFXSimState;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.DoubleSubscriber;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.CAN;
import frc.robot.constants.GROUND.PIVOT;
import frc.robot.constants.ROBOT;
import frc.robot.utils.CtreUtils;

public class GroundPivot extends SubsystemBase {
  /** Creates a new GroundPivot. */
  private final TalonFX m_pivotMotor = new TalonFX(CAN.groundPivotMotor);

  private final TalonFXSimState m_simState = m_pivotMotor.getSimState();

  //private final CANcoder m_pivotEncoder = new CANcoder(CAN.groundPivotCANcoder);
  //private final CANcoderSimState m_pivotEncoderSimState = m_pivotEncoder.getSimState();

  private final StatusSignal<Angle> m_positionSignal = m_pivotMotor.getPosition().clone();
  private final StatusSignal<Current> m_currentSignal = m_pivotMotor.getTorqueCurrent().clone();

  private NeutralModeValue m_neutralMode = NeutralModeValue.Brake;

  private Angle m_desiredAngle = PIVOT.PIVOT_SETPOINT.STOWED.get();

  private final MotionMagicTorqueCurrentFOC m_request =
      new MotionMagicTorqueCurrentFOC(getCurrentAngle());

  // Simulation setup
  private final SingleJointedArmSim m_pivotSim =
      new SingleJointedArmSim(
          PIVOT.gearBox,
          PIVOT.gearRatio,
          SingleJointedArmSim.estimateMOI(PIVOT.pivotLength, PIVOT.mass),
          PIVOT.pivotLength,
          PIVOT.minAngle.in(Radians),
          PIVOT.maxAngle.in(Radians),
          false,
          PIVOT.minAngle.in(Radians));

  private ROBOT.CONTROL_MODE m_controlMode = ROBOT.CONTROL_MODE.CLOSED_LOOP;

  // Test mode setup
  private DoubleSubscriber m_kS_subscriber,
      m_kV_subscriber,
      m_kA_subscriber,
      m_kP_subscriber,
      m_kI_subscriber,
      m_kD_subscriber,
      m_kAccel_subscriber,
      m_kCruiseVel_subscriber,
      m_kJerk_subscriber,
      m_kSetpoint_subscriber;
  private final NetworkTable m_pivotTab =
      NetworkTableInstance.getDefault().getTable("Shuffleboard").getSubTable("GroundPivot");

  public GroundPivot() {
    TalonFXConfiguration config = new TalonFXConfiguration();
    config.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive; // TODO update
    config.MotorOutput.NeutralMode = m_neutralMode;
    config.Feedback.RotorToSensorRatio = PIVOT.gearRatio;
    //config.Feedback.FeedbackRemoteSensorID = CAN.groundPivotCANcoder;
    //config.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.FusedCANcoder;
    config.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.RotorSensor;
    config.Slot0.kS = PIVOT.kS;
    config.Slot0.kV = PIVOT.kV;
    config.Slot0.kP = PIVOT.kP;
    config.Slot0.kI = PIVOT.kI;
    config.Slot0.kD = PIVOT.kD;
    config.ClosedLoopGeneral.ContinuousWrap = false;
    config.MotorOutput.PeakForwardDutyCycle = PIVOT.maxOutput;
    config.MotorOutput.PeakReverseDutyCycle = -PIVOT.maxOutput;

    // Ramp rates TODO: determine if we still need these
    // config.OpenLoopRamps.DutyCycleOpenLoopRampPeriod = 2.0;
    // config.OpenLoopRamps.TorqueOpenLoopRampPeriod = 2.0;
    // config.OpenLoopRamps.VoltageOpenLoopRampPeriod = 2.0;

    config.MotionMagic.MotionMagicAcceleration = PIVOT.kAccel;
    config.MotionMagic.MotionMagicCruiseVelocity = PIVOT.kCruiseVel;
    config.MotionMagic.MotionMagicJerk = PIVOT.kJerk;
    CtreUtils.configureTalonFx(m_pivotMotor, config);

    // CANcoderConfiguration canCoderConfig = new CANcoderConfiguration();
    // canCoderConfig.MagnetSensor.AbsoluteSensorDiscontinuityPoint = 0.5; // TODO update
    // canCoderConfig.MagnetSensor.SensorDirection =
    //     SensorDirectionValue.Clockwise_Positive; // TODO update
    // canCoderConfig.MagnetSensor.MagnetOffset = PIVOT.canCoderOffset;
    // CtreUtils.configureCANCoder(m_pivotEncoder, canCoderConfig);

    // m_pivotEncoder.setPosition(m_pivotEncoder.getAbsolutePosition().getValue());

    // TODO determine if we still need this
    if (RobotBase.isSimulation()) {
      //m_pivotEncoder.setPosition(Units.degreesToRotations(180));
      //m_pivotMotor.setPosition(Units.degreesToRotations(180));
    //   CANcoderConfiguration simCanCoderConfig = new CANcoderConfiguration();
    //   simCanCoderConfig.MagnetSensor.AbsoluteSensorDiscontinuityPoint = 0.5; // TODO update
    //   simCanCoderConfig.MagnetSensor.SensorDirection =
    //       SensorDirectionValue.CounterClockwise_Positive; // TODO update
    //   simCanCoderConfig.MagnetSensor.MagnetOffset = PIVOT.canCoderOffset;
      //CtreUtils.configureCANCoder(m_pivotEncoder, simCanCoderConfig);
    }

    SmartDashboard.putData(this);
  }

  // Get the percent output of the pivot motor.
  public void setPercentOutput(double speed) {
    m_pivotMotor.set(speed);
  }

  public double getPercentOutput() {
    return m_pivotMotor.get();
  }

  public void setDesiredSetpoint(Angle angle) {
    m_desiredAngle =
        Degrees.of(
            MathUtil.clamp(
                angle.in(Degrees), PIVOT.minAngle.in(Degrees), PIVOT.maxAngle.in(Degrees)));
    // Print a stack trace
    System.out.println("Setting desired setpoint to " + Math.round(angle.in(Degrees)) + " degrees");
  }

  public Angle getDesiredSetpoint() {
    return m_desiredAngle;
  }

  public Angle getCurrentAngle() {
    m_positionSignal.refresh();
    return m_positionSignal.getValue();
  }

//   public Angle getCANcoderAngle() {
//     return m_pivotEncoder.getAbsolutePosition().getValue();
//   }

  public void setNeutralMode(NeutralModeValue mode) {
    if (mode == m_neutralMode) return;
    m_neutralMode = mode;
    m_pivotMotor.setNeutralMode(mode);
  }

  public void setControlMode(ROBOT.CONTROL_MODE mode) {
    if (mode == ROBOT.CONTROL_MODE.CLOSED_LOOP && m_controlMode == ROBOT.CONTROL_MODE.OPEN_LOOP)
      resetMotionMagicState();
    m_controlMode = mode;
  }

  public ROBOT.CONTROL_MODE getControlMode() {
    return m_controlMode;
  }

  public void resetSensorPositionHome() {
    resetSensorPosition(PIVOT.startingAngle);
  }

  public void resetSensorPosition(Angle m_angle) {
    m_pivotMotor.setPosition(m_angle);
    resetMotionMagicState();
  }

  public void resetMotionMagicState() {
    m_desiredAngle = getCurrentAngle();
    m_pivotMotor.setControl(m_request.withPosition(m_desiredAngle));
  }

  public TalonFX getMotor() {
    return m_pivotMotor;
  }

  public SingleJointedArmSim getSim() {
    return m_pivotSim;
  }

  public Voltage getInputVoltage() {
    return m_pivotMotor.getMotorVoltage().getValue();
  }

  public AngularVelocity getRotationalVelocity() {
    return m_pivotMotor.getVelocity().getValue();
  }

  private void updateLogger() {
    SmartDashboard.putString("GroundPivot/ControlMode", m_controlMode.toString());
    SmartDashboard.putNumber("GroundPivot/CurrentAngle", getCurrentAngle().in(Degrees));
    SmartDashboard.putNumber("GroundPivot/CurrentOutput", m_currentSignal.getValueAsDouble());
    SmartDashboard.putNumber("GroundPivot/DesiredAngle", m_desiredAngle.in(Degrees));
    SmartDashboard.putNumber("GroundPivot/PercentOutput", m_pivotMotor.get());
    //SmartDashboard.putNumber("GroundPivot/CanCoderAbsolutePos360", getCANcoderAngle().in(Degrees));
  }

  public void testInit() {
    m_pivotTab.getDoubleTopic("kS").publish().set(PIVOT.kS);
    m_pivotTab.getDoubleTopic("kV").publish().set(PIVOT.kV);
    m_pivotTab.getDoubleTopic("kA").publish().set(PIVOT.kA);
    m_pivotTab.getDoubleTopic("kP").publish().set(PIVOT.kP);
    m_pivotTab.getDoubleTopic("kI").publish().set(PIVOT.kI);
    m_pivotTab.getDoubleTopic("kD").publish().set(PIVOT.kD);

    m_pivotTab.getDoubleTopic("kAccel").publish().set(PIVOT.kAccel);
    m_pivotTab.getDoubleTopic("kCruiseVel").publish().set(PIVOT.kCruiseVel);
    m_pivotTab.getDoubleTopic("kJerk").publish().set(PIVOT.kJerk);

    m_pivotTab.getDoubleTopic("kSetpoint").publish().set(getCurrentAngle().in(Degrees));

    m_kS_subscriber = m_pivotTab.getDoubleTopic("kS").subscribe(PIVOT.kS);
    m_kV_subscriber = m_pivotTab.getDoubleTopic("kV").subscribe(PIVOT.kV);
    m_kA_subscriber = m_pivotTab.getDoubleTopic("kA").subscribe(PIVOT.kA);
    m_kP_subscriber = m_pivotTab.getDoubleTopic("kP").subscribe(PIVOT.kP);
    m_kI_subscriber = m_pivotTab.getDoubleTopic("kI").subscribe(PIVOT.kI);
    m_kD_subscriber = m_pivotTab.getDoubleTopic("kD").subscribe(PIVOT.kD);

    m_kAccel_subscriber = m_pivotTab.getDoubleTopic("kAccel").subscribe(PIVOT.kAccel);
    m_kCruiseVel_subscriber = m_pivotTab.getDoubleTopic("kCruiseVel").subscribe(PIVOT.kCruiseVel);
    m_kJerk_subscriber = m_pivotTab.getDoubleTopic("kJerk").subscribe(PIVOT.kJerk);

    m_kSetpoint_subscriber =
        m_pivotTab.getDoubleTopic("kSetpoint").subscribe(m_desiredAngle.in(Degrees));
  }

  public void testPeriodic() {
    Slot0Configs slot0Configs = new Slot0Configs();
    slot0Configs.kS = m_kS_subscriber.get(PIVOT.kS);
    slot0Configs.kV = m_kV_subscriber.get(PIVOT.kV);
    slot0Configs.kA = m_kA_subscriber.get(PIVOT.kA);
    slot0Configs.kP = m_kP_subscriber.get(PIVOT.kP);
    slot0Configs.kI = m_kI_subscriber.get(PIVOT.kI);
    slot0Configs.kD = m_kD_subscriber.get(PIVOT.kD);

    m_pivotMotor.getConfigurator().apply(slot0Configs);

    MotionMagicConfigs motionMagicConfigs = new MotionMagicConfigs();

    motionMagicConfigs.MotionMagicAcceleration = m_kAccel_subscriber.get(PIVOT.kAccel);
    motionMagicConfigs.MotionMagicCruiseVelocity = m_kCruiseVel_subscriber.get(PIVOT.kCruiseVel);
    motionMagicConfigs.MotionMagicJerk = m_kJerk_subscriber.get(PIVOT.kJerk);

    m_pivotMotor.getConfigurator().apply(motionMagicConfigs);

    Angle m_oldSetpoint = m_desiredAngle;
    m_desiredAngle = Degrees.of(m_kSetpoint_subscriber.get(m_desiredAngle.in(Degrees)));
    if (m_desiredAngle.equals(m_oldSetpoint)) setDesiredSetpoint(m_desiredAngle);
  }

  public void autonomousInit() {
    resetMotionMagicState();
    setDesiredSetpoint(getCurrentAngle());
  }

  public void teleopInit() {
    resetMotionMagicState();
    setDesiredSetpoint(getCurrentAngle());
  }

  @Override
  public void periodic() {
    switch (m_controlMode) {
      case CLOSED_LOOP:
        // This method will be called once per scheduler run
        // periodic, update the profile setpoint for 20 ms loop time
        if (DriverStation.isEnabled())
          m_pivotMotor.setControl(m_request.withPosition(m_desiredAngle));
        break;
      default:
      case OPEN_LOOP:
        if (DriverStation.isDisabled()) {
          setPercentOutput(0.0);
        }
        break;
    }

    if (ROBOT.logMode.get() <= ROBOT.LOG_MODE.NORMAL.get()) updateLogger();
  }

  @Override
  public void simulationPeriodic() {
    // Set supply voltage of pivot motor
    m_simState.setSupplyVoltage(RobotController.getBatteryVoltage());

    m_pivotSim.setInputVoltage(MathUtil.clamp(m_simState.getMotorVoltage(), -12, 12));

    m_pivotSim.update(0.020);

    m_simState.setRawRotorPosition(
        Units.radiansToRotations(m_pivotSim.getAngleRads()) * PIVOT.gearRatio);

    m_simState.setRotorVelocity(
        Units.radiansToRotations(m_pivotSim.getVelocityRadPerSec()) * PIVOT.gearRatio);

    //m_pivotEncoderSimState.setRawPosition(Units.radiansToRotations(m_pivotSim.getAngleRads()));
    //m_pivotEncoderSimState.setVelocity(Units.radiansToRotations(m_pivotSim.getVelocityRadPerSec()));

    SmartDashboard.putNumber(
        "GroundPivot/Model Angle", Units.radiansToDegrees(m_pivotSim.getAngleRads()));
  }
}
