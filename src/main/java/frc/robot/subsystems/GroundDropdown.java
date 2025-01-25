package frc.robot.subsystems;

import java.util.logging.Logger;

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
import frc.robot.constants.GROUND.DROPDOWN;
import frc.robot.constants.ROBOT;
import frc.robot.utils.CtreUtils;

public class GroundDropdown extends SubsystemBase {
/** Creates a new GroundDropdown. */
 private final TalonFX m_dropdownMotor = new TalonFX(CAN.groundDropdownMotor);

 private final TalonFXSimState m_simState = m_dropdownMotor.getSimState();

 private final CANcoder m_dropdownEncoder = new CANcoder(CAN.groundDropdownCANcoder);
 private final CANcoderSimState m_dropdownEncoderSimState = m_dropdownEncoder.getSimState();

 private final StatusSignal<Angle> m_positionSignal = m_dropdownMotor.getPosition().clone();
 private final StatusSignal<Current> m_currentSignal = m_dropdownMotor.getTorqueCurrent().clone();

 private NeutralModeValue m_neutralMode = NeutralModeValue.Brake;

 private double m_desiredRotations = DROPDOWN.DROPDOWN_SETPOINT.STOWED.get();

 private final MotionMagicTorqueCurrentFOC m_request =
     new MotionMagicTorqueCurrentFOC(getCurrentRotation());

 // Simulation setup
 private final SingleJointedArmSim m_dropdownSim =
     new SingleJointedArmSim(
         DROPDOWN.gearBox,
         DROPDOWN.gearRatio,
         SingleJointedArmSim.estimateMOI(DROPDOWN.dropdownLength, DROPDOWN.mass),
         DROPDOWN.dropdownLength,
         Units.degreesToRadians(DROPDOWN.minAngleDegrees),
         Units.degreesToRadians(DROPDOWN.maxAngleDegrees),
         false,
         Units.degreesToRadians(DROPDOWN.minAngleDegrees));

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
 private final NetworkTable dropdownTab =
     NetworkTableInstance.getDefault().getTable("Shuffleboard").getSubTable("GroundDropdown");

 public GroundDropdown() {
   TalonFXConfiguration config = new TalonFXConfiguration();
   config.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive; // TODO update
   config.MotorOutput.NeutralMode = m_neutralMode;
   config.Feedback.RotorToSensorRatio = DROPDOWN.gearRatio;
   config.Feedback.FeedbackRemoteSensorID = CAN.groundDropdownCANcoder;
   config.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.FusedCANcoder;
   config.Slot0.kS = DROPDOWN.kS;
   config.Slot0.kV = DROPDOWN.kV;
   config.Slot0.kP = DROPDOWN.kP;
   config.Slot0.kI = DROPDOWN.kI;
   config.Slot0.kD = DROPDOWN.kD;
   config.ClosedLoopGeneral.ContinuousWrap = false;
   config.MotorOutput.PeakForwardDutyCycle = DROPDOWN.maxOutput;
   config.MotorOutput.PeakReverseDutyCycle = -DROPDOWN.maxOutput;

   // Ramp rates TODO: determine if we still need these
   config.OpenLoopRamps.DutyCycleOpenLoopRampPeriod = 2.0;
   config.OpenLoopRamps.TorqueOpenLoopRampPeriod = 2.0;
   config.OpenLoopRamps.VoltageOpenLoopRampPeriod = 2.0;

   config.MotionMagic.MotionMagicAcceleration = DROPDOWN.kAccel;
   config.MotionMagic.MotionMagicCruiseVelocity = DROPDOWN.kCruiseVel;
   config.MotionMagic.MotionMagicJerk = DROPDOWN.kJerk;
   CtreUtils.configureTalonFx(m_dropdownMotor, config);

   CANcoderConfiguration canCoderConfig = new CANcoderConfiguration();
   canCoderConfig.MagnetSensor.AbsoluteSensorDiscontinuityPoint = 0.5; // TODO update
   canCoderConfig.MagnetSensor.SensorDirection = SensorDirectionValue.Clockwise_Positive; // TODO update
   canCoderConfig.MagnetSensor.MagnetOffset = DROPDOWN.canCoderOffset;
   CtreUtils.configureCANCoder(m_dropdownEncoder, canCoderConfig);

   m_dropdownEncoder.setPosition(m_dropdownEncoder.getAbsolutePosition().getValue());

   // TODO determine if we still need this
   if (RobotBase.isSimulation()) {
     m_dropdownEncoder.setPosition(Units.degreesToRotations(180));
     m_dropdownMotor.setPosition(Units.degreesToRotations(180));
     CANcoderConfiguration simCanCoderConfig = new CANcoderConfiguration();
     simCanCoderConfig.MagnetSensor.AbsoluteSensorDiscontinuityPoint = 0.5; // TODO update
     simCanCoderConfig.MagnetSensor.SensorDirection =
         SensorDirectionValue.CounterClockwise_Positive;
     simCanCoderConfig.MagnetSensor.MagnetOffset = DROPDOWN.canCoderOffset;
     CtreUtils.configureCANCoder(m_dropdownEncoder, simCanCoderConfig);
   }

   SmartDashboard.putData(this);
 }

 // Get the percent output of the dropdown motor.
 public void setPercentOutput(double speed) {
   m_dropdownMotor.set(speed);
 }

 public double getPercentOutput() {
   return m_dropdownMotor.get();
 }

 public double setDesiredSetpointRotations(double rotations) {
   m_desiredRotations =
       MathUtil.clamp(
           rotations,
           Units.degreesToRotations(DROPDOWN.minAngleDegrees),
           Units.degreesToRotations(DROPDOWN.maxAngleDegrees));
 }

 public double getDesiredSetpointRotations() {
   return m_desiredRotations;
 }

 public Angle getCurrentRotation() {
   m_positionSignal.refresh();
   return m_positionSignal.getValue();
 }

 public double getCurrentAngle() {
   return Units.rotationsToDegrees(getCurrentRotation());
 }

 public double getCANcoderAngle() {
   return m_dropdownEncoder.getAbsolutePosition().getValueAsDouble() * 360;
 }

 public void setNeutralMode(NeutralModeValue mode) {
   if (mode == m_neutralMode) return;
   m_neutralMode = mode;
   m_dropdownMotor.setNeutralMode(mode);
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
   resetSensorPosition(DROPDOWN.startingAngleDegrees);
 }

 public void resetSensorPosition(double m_angle) {
   m_dropdownMotor.setPosition(Units.degreesToRotations(m_angle));
   resetMotionMagicState();
 }

 public void resetMotionMagicState() {
   m_desiredRotations = getCurrentRotation();
   m_dropdownMotor.setControl(m_request.withPosition(m_desiredRotations));
 }

 public TalonFX getMotor() {
   return m_dropdownMotor;
 }

 public SingleJointedArmSim getSim() {
   return m_dropdownSim;
 }

 public Voltage getInputVoltage() {
   return m_dropdownMotor.getMotorVoltage().getValue();
 }

 public AngularVelocity getRotationalVelocity() {
   return m_dropdownMotor.getVelocity().getValue();
 }

 private void updateLogger() {
   Logger.recordOutput("GroundDropdown/ControlMode", m_controlMode.toString());
   Logger.recordOutput("GroundDropdown/CurrentAngle", getCurrentAngle());
   Logger.recordOutput("GroundDropdown/CurrentOutput", m_currentSignal.getValue());
   Logger.recordOutput("GroundDropdown/DesiredAngle", Units.rotationsToDegrees(m_desiredRotations));
   Logger.recordOutput("GroundDropdown/PercentOutput", m_dropdownMotor.get());
   Logger.recordOutput("GroundDropdown/CanCoderAbsolutePos360", getCANcoderAngle());
 }

 public void testInit() {
   dropdownTab.getDoubleTopic("kS").publish().set(DROPDOWN.kS);
   dropdownTab.getDoubleTopic("kV").publish().set(DROPDOWN.kV);
   dropdownTab.getDoubleTopic("kA").publish().set(DROPDOWN.kA);
   dropdownTab.getDoubleTopic("kP").publish().set(DROPDOWN.kP);
   dropdownTab.getDoubleTopic("kI").publish().set(DROPDOWN.kI);
   dropdownTab.getDoubleTopic("kD").publish().set(DROPDOWN.kD);

   dropdownTab.getDoubleTopic("kAccel").publish().set(DROPDOWN.kAccel);
   dropdownTab.getDoubleTopic("kCruiseVel").publish().set(DROPDOWN.kCruiseVel);
   dropdownTab.getDoubleTopic("kJerk").publish().set(DROPDOWN.kJerk);

   dropdownTab.getDoubleTopic("kSetpoint").publish().set(getCurrentAngle());

   m_kS_subscriber = dropdownTab.getDoubleTopic("kS").subscribe(DROPDOWN.kS);
   m_kV_subscriber = dropdownTab.getDoubleTopic("kV").subscribe(DROPDOWN.kV);
   m_kA_subscriber = dropdownTab.getDoubleTopic("kA").subscribe(DROPDOWN.kA);
   m_kP_subscriber = dropdownTab.getDoubleTopic("kP").subscribe(DROPDOWN.kP);
   m_kI_subscriber = dropdownTab.getDoubleTopic("kI").subscribe(DROPDOWN.kI);
   m_kD_subscriber = dropdownTab.getDoubleTopic("kD").subscribe(DROPDOWN.kD);

   m_kAccel_subscriber = dropdownTab.getDoubleTopic("kAccel").subscribe(DROPDOWN.kAccel);
   m_kCruiseVel_subscriber = dropdownTab.getDoubleTopic("kCruiseVel").subscribe(DROPDOWN.kCruiseVel);
   m_kJerk_subscriber = dropdownTab.getDoubleTopic("kJerk").subscribe(DROPDOWN.kJerk);

   m_kSetpoint_subscriber =
       dropdownTab.getDoubleTopic("kSetpoint").subscribe(Units.rotationsToDegrees(m_desiredRotations));
 }

 public void testPeriodic() {
   Slot0Configs slot0Configs = new Slot0Configs();
   slot0Configs.kS = m_kS_subscriber.get(DROPDOWN.kS);
   slot0Configs.kV = m_kV_subscriber.get(DROPDOWN.kV);
   slot0Configs.kA = m_kA_subscriber.get(DROPDOWN.kA);
   slot0Configs.kP = m_kP_subscriber.get(DROPDOWN.kP);
   slot0Configs.kI = m_kI_subscriber.get(DROPDOWN.kI);
   slot0Configs.kD = m_kD_subscriber.get(DROPDOWN.kD);

   m_dropdownMotor.getConfigurator().apply(slot0Configs);

   MotionMagicConfigs motionMagicConfigs = new MotionMagicConfigs();

   motionMagicConfigs.MotionMagicAcceleration = m_kAccel_subscriber.get(DROPDOWN.kAccel);
   motionMagicConfigs.MotionMagicCruiseVelocity = m_kCruiseVel_subscriber.get(DROPDOWN.kCruiseVel);
   motionMagicConfigs.MotionMagicJerk = m_kJerk_subscriber.get(DROPDOWN.kJerk);

   m_dropdownMotor.getConfigurator().apply(motionMagicConfigs);

   double m_oldSetpoint = Units.rotationsToDegrees(m_desiredRotations);
   m_desiredRotations =
       Units.degreesToRotations(
           m_kSetpoint_subscriber.get(Units.rotationsToDegrees(m_desiredRotations)));
   if (m_desiredRotations != m_oldSetpoint) setDesiredSetpointRotations(m_desiredRotations);
 }

 public void autonomousInit() {
   resetMotionMagicState();
   setDesiredSetpointRotations(getCurrentRotation());
 }

 public void teleopInit() {
   resetMotionMagicState();
   setDesiredSetpointRotations(getCurrentRotation());
 }

 @Override
 public void periodic() {
   switch (m_controlMode) {
     case CLOSED_LOOP:
       // This method will be called once per scheduler run
       // periodic, update the profile setpoint for 20 ms loop time
       if (DriverStation.isEnabled())
         m_dropdownMotor.setControl(m_request.withPosition(m_desiredRotations));
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
   //    // Set supply voltage of flipper motor
   m_simState.setSupplyVoltage(RobotController.getBatteryVoltage());

   m_dropdownSim.setInputVoltage(MathUtil.clamp(m_simState.getMotorVoltage(), -12, 12));

   m_dropdownSim.update(RobotTime.getTimeDelta());

   m_simState.setRawRotorPosition(
       Units.radiansToRotations(m_dropdownSim.getAngleRads()) * DROPDOWN.gearRatio);

   m_simState.setRotorVelocity(
       Units.radiansToRotations(m_dropdownSim.getVelocityRadPerSec()) * DROPDOWN.gearRatio);

   m_dropdownEncoderSimState.setRawPosition(Units.radiansToRotations(m_dropdownSim.getAngleRads()));
   m_dropdownEncoderSimState.setVelocity(Units.radiansToRotations(m_dropdownSim.getVelocityRadPerSec()));

   Logger.recordOutput("GroundDropdown/Model Angle", Units.radiansToDegrees(m_dropdownSim.getAngleRads()));
 }
}