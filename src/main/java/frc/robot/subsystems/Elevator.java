// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.*;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.sim.TalonFXSimState;
import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.DoubleSubscriber;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.units.measure.*;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.simulation.ElevatorSim;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.CAN;
import frc.robot.constants.ELEVATOR;
import frc.robot.constants.ELEVATOR.ELEVATOR_SETPOINT;
import frc.robot.constants.ROBOT.CONTROL_MODE;
import org.team4201.codex.utils.CtreUtils;

public class Elevator extends SubsystemBase {
  /** Creates a new Elevator */
  private final TalonFX[] elevatorMotors = {
    new TalonFX(CAN.elevatorMotor1), new TalonFX(CAN.elevatorMotor2)
  };

  private final StatusSignal<Angle> m_positionSignal = elevatorMotors[0].getPosition().clone();
  private final StatusSignal<AngularVelocity> m_velocitySignal =
      elevatorMotors[0].getVelocity().clone();
  private final StatusSignal<AngularAcceleration> m_accelSignal =
      elevatorMotors[0].getAcceleration().clone();
  private final StatusSignal<Voltage> m_voltageSignal = elevatorMotors[0].getMotorVoltage().clone();
  private final StatusSignal<Current> m_supplyCurrentSignal =
      elevatorMotors[0].getSupplyCurrent().clone();
  private final StatusSignal<Current> m_statorCurrentSignal =
      elevatorMotors[0].getStatorCurrent().clone();
  private final StatusSignal<Current> m_torqueCurrentSignal =
      elevatorMotors[0].getTorqueCurrent().clone();

  private Distance m_desiredPosition = Inches.of(0);

  @Logged(name = "Joystick Input", importance = Logged.Importance.DEBUG)
  private double m_joystickInput = 0.0;

  @Logged(name = "Control Mode", importance = Logged.Importance.INFO)
  private CONTROL_MODE m_controlMode = CONTROL_MODE.OPEN_LOOP;

  @Logged(name = "Neutral Mode", importance = Logged.Importance.INFO)
  private NeutralModeValue m_neutralMode = NeutralModeValue.Brake;

  //  private final MotionMagicVoltage m_request = new MotionMagicVoltage(0).withEnableFOC(true);
  private final MotionMagicTorqueCurrentFOC m_request = new MotionMagicTorqueCurrentFOC(0);

  //  private final MotionMagicVelocityVoltage m_requestVelocity = new
  // MotionMagicVelocityVoltage(0).withEnableFOC(true);
  private final MotionMagicVelocityTorqueCurrentFOC m_requestVelocity =
      new MotionMagicVelocityTorqueCurrentFOC(0);

  private DoubleSubscriber m_kG_subscriber,
      m_kS_subscriber,
      m_kV_Subscriber,
      m_kA_Subscriber,
      m_kP_subscriber,
      m_kI_subscriber,
      m_kD_subscriber,
      m_velocitySubscriber,
      m_accelerationSubscriber,
      m_setpointSubscriber;
  private final NetworkTable elevatorTab =
      NetworkTableInstance.getDefault().getTable("Shuffleboard").getSubTable("Elevator");

  // Simulation classes help us simulate what's going on, including gravity.
  private final ElevatorSim m_elevatorSim =
      new ElevatorSim(
          ELEVATOR.gearbox,
          ELEVATOR.gearRatio,
          ELEVATOR.kCarriageMass.in(Kilograms),
          ELEVATOR.kElevatorDrumDiameter.div(2).in(Meters),
          ELEVATOR.lowerLimit.in(Meters),
          ELEVATOR.upperLimit.in(Meters),
          true,
          1);
  private final TalonFXSimState m_motorSimState;

  public Elevator() {
    TalonFXConfiguration config = new TalonFXConfiguration();
    config.Slot0.kG = ELEVATOR.kG;
    config.Slot0.kS = ELEVATOR.kS;
    config.Slot0.kV = ELEVATOR.kV;
    config.Slot0.kA = ELEVATOR.kA;
    config.Slot0.kP = ELEVATOR.kP;
    config.Slot0.kI = ELEVATOR.kI;
    config.Slot0.kD = ELEVATOR.kD;

    config.Feedback.SensorToMechanismRatio = ELEVATOR.gearRatio;
    config.MotionMagic.MotionMagicCruiseVelocity = ELEVATOR.motionMagicCruiseVelocity;
    config.MotionMagic.MotionMagicAcceleration = ELEVATOR.motionMagicAcceleration;
    config.MotionMagic.MotionMagicJerk = ELEVATOR.motionMagicJerk;
    if (!RobotBase.isSimulation()) config.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
    //    config.MotorOutput.PeakReverseDutyCycle = ELEVATOR.peakReverseOutput;
    //    config.MotorOutput.PeakForwardDutyCycle = ELEVATOR.peakForwardOutput;
    config.MotorOutput.NeutralMode = m_neutralMode;

    CtreUtils.configureTalonFx(elevatorMotors[0], config);
    CtreUtils.configureTalonFx(elevatorMotors[1], config);

    m_motorSimState = elevatorMotors[0].getSimState();

    elevatorMotors[0].setPosition(Rotations.of(0));
    elevatorMotors[1].setControl(new Follower(elevatorMotors[0].getDeviceID(), true));

    setName("Elevator");
    SmartDashboard.putData(this);
  }

  public void holdElevator() {
    setDesiredPosition(getHeight());
  }

  public void setPercentOutput(double output) {
    elevatorMotors[0].set(output);
  }

  @Logged(name = "Motor Output", importance = Logged.Importance.INFO)
  public double getPercentOutput() {
    return elevatorMotors[0].get();
  }

  public void setDesiredPosition(Distance desiredPosition) {
    m_desiredPosition = desiredPosition;
  }

  public void setDesiredAcceleration(double desiredAccel) {
    m_requestVelocity.Acceleration = desiredAccel;
  }

  public void setJoystickInput(double joystickInput) {
    m_joystickInput = joystickInput;
  }

  public void setControlMode(CONTROL_MODE controlMode) {
    m_controlMode = controlMode;
  }

  public CONTROL_MODE getControlMode() {
    return m_controlMode;
  }

  @Logged(name = "Motor Rotations", importance = Logged.Importance.DEBUG)
  public Angle getRotations() {
    return m_positionSignal.refresh().getValue();
  }

  @Logged(name = "Motor Velocity", importance = Logged.Importance.DEBUG)
  public AngularVelocity getMotorVelocity() {
    return m_velocitySignal.refresh().getValue();
  }

  @Logged(name = "Motor Acceleration", importance = Logged.Importance.DEBUG)
  public AngularAcceleration getMotorAcceleration() {
    return m_accelSignal.refresh().getValue();
  }

  @Logged(name = "Motor Reference", importance = Logged.Importance.DEBUG)
  public Angle getReference() {
    return Rotations.of(elevatorMotors[0].getClosedLoopReference().refresh().getValue());
  }

  public LinearVelocity getVelocity() {
    return MetersPerSecond.of(
        getMotorVelocity().in(RotationsPerSecond) * ELEVATOR.drumRotationsToDistance.in(Meters));
  }

  @Logged(name = "Velocity Inches-s", importance = Logged.Importance.INFO)
  public double getVelocityInches() {
    return getVelocity().in(InchesPerSecond);
  }

  public LinearAcceleration getAcceleration() {
    return MetersPerSecondPerSecond.of(
        getMotorAcceleration().in(RotationsPerSecondPerSecond)
            * ELEVATOR.drumRotationsToDistance.in(Meters));
  }

  @Logged(name = "Acceleration Inches-s^2", importance = Logged.Importance.INFO)
  public double getAccelerationInches() {
    return Units.metersToInches(getAcceleration().in(MetersPerSecondPerSecond));
  }

  @Logged(name = "Motor Voltage", importance = Logged.Importance.DEBUG)
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

  @Logged(name = "Control Request", importance = Logged.Importance.DEBUG)
  public String getCurrentControlRequestString() {
    return elevatorMotors[0].getAppliedControl().toString();
  }

  public Distance getHeight() {
    return ELEVATOR.drumRotationsToDistance.times(getRotations().magnitude());
  }

  @Logged(name = "Height Inches", importance = Logged.Importance.INFO)
  public double getHeightInches() {
    return getHeight().in(Inches);
  }

  public boolean isClosedLoopControl() {
    return getControlMode() == CONTROL_MODE.CLOSED_LOOP;
  }

  public void setNeutralMode(NeutralModeValue mode) {
    if (mode == m_neutralMode) return;
    m_neutralMode = mode;
    elevatorMotors[0].setNeutralMode(mode);
  }

  public NeutralModeValue getNeutralMode() {
    return m_neutralMode;
  }

  @Logged(name = "Desired Height Inches", importance = Logged.Importance.INFO)
  public double getDesiredHeightInches() {
    return m_desiredPosition.in(Inches);
  }

  // Elevator is within 1 inch of its setpoint
  @Logged(name = "At Setpoint", importance = Logged.Importance.DEBUG)
  public boolean atSetpoint() {
    return m_desiredPosition.minus(getHeight()).abs(Inches) <= 1; // RIP the 254 reference
  }

  public boolean[] isConnected() {
    return new boolean[] {elevatorMotors[0].isConnected(), elevatorMotors[1].isConnected()};
  }

  public void testInit() {
    elevatorTab.getDoubleTopic("kG").publish().set(ELEVATOR.kG);
    elevatorTab.getDoubleTopic("kS").publish().set(ELEVATOR.kS);
    elevatorTab.getDoubleTopic("kV").publish().set(ELEVATOR.kV);
    elevatorTab.getDoubleTopic("kA").publish().set(ELEVATOR.kA);
    elevatorTab.getDoubleTopic("kP").publish().set(ELEVATOR.kP);
    elevatorTab.getDoubleTopic("kI").publish().set(ELEVATOR.kI);
    elevatorTab.getDoubleTopic("kD").publish().set(ELEVATOR.kD);
    elevatorTab
        .getDoubleTopic("MotionMagicCruiseVelocity")
        .publish()
        .set(ELEVATOR.motionMagicCruiseVelocity);
    elevatorTab
        .getDoubleTopic("MotionMagicAcceleration")
        .publish()
        .set(ELEVATOR.motionMagicAcceleration);
    elevatorTab.getDoubleTopic("Setpoint Inches").publish().set(m_desiredPosition.in(Inches));
    m_kG_subscriber = elevatorTab.getDoubleTopic("kG").subscribe(ELEVATOR.kG);
    m_kS_subscriber = elevatorTab.getDoubleTopic("kS").subscribe(ELEVATOR.kS);
    m_kV_Subscriber = elevatorTab.getDoubleTopic("kV").subscribe(ELEVATOR.kV);
    m_kA_Subscriber = elevatorTab.getDoubleTopic("kA").subscribe(ELEVATOR.kA);
    m_kP_subscriber = elevatorTab.getDoubleTopic("kP").subscribe(ELEVATOR.kP);
    m_kI_subscriber = elevatorTab.getDoubleTopic("kI").subscribe(ELEVATOR.kI);
    m_kD_subscriber = elevatorTab.getDoubleTopic("kD").subscribe(ELEVATOR.kD);
    m_velocitySubscriber =
        elevatorTab
            .getDoubleTopic("MotionMagicCruiseVelocity")
            .subscribe(ELEVATOR.motionMagicCruiseVelocity);
    m_accelerationSubscriber =
        elevatorTab
            .getDoubleTopic("MotionMagicAcceleration")
            .subscribe(ELEVATOR.motionMagicAcceleration);

    m_setpointSubscriber =
        elevatorTab.getDoubleTopic("Setpoint Inches").subscribe(m_desiredPosition.in(Inches));
  }

  public void testPeriodic() {
    Slot0Configs slot0Configs = new Slot0Configs();

    slot0Configs.kG = m_kG_subscriber.get(ELEVATOR.kG);
    slot0Configs.kS = m_kS_subscriber.get(ELEVATOR.kS);
    slot0Configs.kV = m_kV_Subscriber.get(ELEVATOR.kV);
    slot0Configs.kA = m_kA_Subscriber.get(ELEVATOR.kA);
    slot0Configs.kP = m_kP_subscriber.get(ELEVATOR.kP);
    slot0Configs.kI = m_kI_subscriber.get(ELEVATOR.kI);
    slot0Configs.kD = m_kD_subscriber.get(ELEVATOR.kD);

    elevatorMotors[0].getConfigurator().apply(slot0Configs);
    elevatorMotors[1].getConfigurator().apply(slot0Configs);

    MotionMagicConfigs motionMagicConfigs = new MotionMagicConfigs();

    motionMagicConfigs.MotionMagicCruiseVelocity =
        m_velocitySubscriber.get(ELEVATOR.motionMagicCruiseVelocity);
    motionMagicConfigs.MotionMagicAcceleration =
        m_accelerationSubscriber.get(ELEVATOR.motionMagicAcceleration);

    elevatorMotors[0].getConfigurator().apply(motionMagicConfigs);
    elevatorMotors[1].getConfigurator().apply(motionMagicConfigs);

    double newSetpoint = m_setpointSubscriber.get(m_desiredPosition.in(Inches));
    if (newSetpoint != m_desiredPosition.in(Inches)) {
      setDesiredPosition(Inches.of(newSetpoint));
    }
  }

  public void teleopInit() {
    holdElevator();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    switch (m_controlMode) {
      case CLOSED_LOOP_NET:
        // m_requestVelocity.Acceleration = 100; TODO: figure out where to put this.
        elevatorMotors[0].setControl(m_requestVelocity.withVelocity(80));
        break;
      case CLOSED_LOOP:
        if (atSetpoint()
            && m_desiredPosition.minus(ELEVATOR_SETPOINT.START_POSITION.getSetpoint()).abs(Inches)
                <= 1) {
          elevatorMotors[0].set(0); // Don't move the elevator if already at stowed
        } else {
          elevatorMotors[0].setControl(
              m_request.withPosition(
                  m_desiredPosition.in(Meters) / ELEVATOR.drumRotationsToDistance.in(Meters)));
        }
        break;
      case OPEN_LOOP:
      default:
        double percentOutput;
        if (m_joystickInput < 0) {
          percentOutput =
              m_joystickInput * ELEVATOR.kLimitedPercentOutputMultiplier + ELEVATOR.offset;
        } else {
          percentOutput = m_joystickInput * ELEVATOR.kPercentOutputMultiplier + ELEVATOR.offset;
        }
        setPercentOutput(percentOutput);
        break;
    }
  }

  @Override
  public void simulationPeriodic() {
    m_motorSimState.setSupplyVoltage(RobotController.getBatteryVoltage());
    m_elevatorSim.setInputVoltage(m_motorSimState.getMotorVoltage());

    m_elevatorSim.update(0.020);

    m_motorSimState.setRawRotorPosition(
        m_elevatorSim.getPositionMeters()
            * ELEVATOR.gearRatio
            / ELEVATOR.drumRotationsToDistance.in(Meters));
    m_motorSimState.setRotorVelocity(
        m_elevatorSim.getVelocityMetersPerSecond()
            * ELEVATOR.gearRatio
            / ELEVATOR.drumRotationsToDistance.in(Meters));
  }
}
