package frc.robot.utils;

import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.constants.CAN;

// TODO: This class is also a mess
public final class CtreUtils {
  /**
   * Initialize Phoenix Server by creating a dummy device. We do this so that the CANCoders don't
   * get configured before Phoenix Server is up, which causes issues with encoder offsets not being
   * set/applied properly.
   */
  public static void initPhoenixServer() {
    var alert =
        new Alert("Starting Phoenix Server at: " + Timer.getFPGATimestamp(), AlertType.kInfo);
    alert.set(true);
    if (RobotBase.isReal()) {
      TalonFX dummy = new TalonFX(0, CAN.driveBaseCanbus);
      Timer.delay(5);
      dummy.close();
      dummy = null;
    }
    alert.setText("Phoenix Server finished Init at: " + Timer.getFPGATimestamp());
  }

  //   public static TalonFXConfiguration generateTurnMotorConfig() {
  //     TalonFXConfiguration turnMotorConfig = new TalonFXConfiguration();

  //     turnMotorConfig.CustomParams.CustomParam0 = 1; // Identify the config

  //     turnMotorConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
  //     turnMotorConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;

  //     turnMotorConfig.Slot0 = SWERVE.MODULE.turnGains;
  //     turnMotorConfig.CurrentLimits.SupplyCurrentLimit = 25;
  //     turnMotorConfig.CurrentLimits.SupplyCurrentThreshold = 40;
  //     turnMotorConfig.CurrentLimits.SupplyTimeThreshold = 0.1;
  //     turnMotorConfig.CurrentLimits.SupplyCurrentLimitEnable = true;

  //     turnMotorConfig.MotionMagic.MotionMagicCruiseVelocity =
  //         100.0 / SWERVE.MODULE.kTurnMotorGearRatio;
  //     turnMotorConfig.MotionMagic.MotionMagicAcceleration =
  //         turnMotorConfig.MotionMagic.MotionMagicCruiseVelocity / 0.100;
  //     turnMotorConfig.MotionMagic.MotionMagicExpo_kV = 0.12 * SWERVE.MODULE.kTurnMotorGearRatio;
  //     turnMotorConfig.MotionMagic.MotionMagicExpo_kA = 0.1;

  //     turnMotorConfig.ClosedLoopGeneral.ContinuousWrap = true;
  //     //    turnMotorConfig.Feedback.FeedbackSensorSource =
  // FeedbackSensorSourceValue.RotorSensor;
  //     turnMotorConfig.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.RemoteCANcoder;
  //     //    turnMotorConfig.Feedback.SensorToMechanismRatio = SWERVE.MODULE.kTurnMotorGearRatio;
  //     turnMotorConfig.Feedback.RotorToSensorRatio = SWERVE.MODULE.kTurnMotorGearRatio;
  //     //    turnMotorConfig.Feedback.FeedbackRemoteSensorID = 0;

  //     return turnMotorConfig;
  //   }

  //   public static TalonFXConfiguration generateDriveMotorConfig() {
  //     TalonFXConfiguration driveMotorConfig = new TalonFXConfiguration();

  //     driveMotorConfig.CustomParams.CustomParam0 = 2; // Identify the config

  //     driveMotorConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
  //     driveMotorConfig.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;

  //     driveMotorConfig.Slot0 = SWERVE.MODULE.driveGains;
  //     driveMotorConfig.TorqueCurrent.PeakForwardTorqueCurrent = SWERVE.MODULE.kSlipCurrent;
  //     driveMotorConfig.TorqueCurrent.PeakReverseTorqueCurrent = -SWERVE.MODULE.kSlipCurrent;
  //     driveMotorConfig.CurrentLimits.StatorCurrentLimit = SWERVE.MODULE.kSlipCurrent;
  //     driveMotorConfig.CurrentLimits.StatorCurrentLimitEnable = true;
  //     driveMotorConfig.CurrentLimits.SupplyCurrentLimit = 35;
  //     driveMotorConfig.CurrentLimits.SupplyCurrentThreshold = 60;
  //     driveMotorConfig.CurrentLimits.SupplyTimeThreshold = 0.1;
  //     driveMotorConfig.CurrentLimits.SupplyCurrentLimitEnable = true;

  //     driveMotorConfig.Feedback.SensorToMechanismRatio = SWERVE.MODULE.kDriveMotorGearRatio;

  //     driveMotorConfig.OpenLoopRamps.DutyCycleOpenLoopRampPeriod = 0.25; // TODO adjust this
  // later
  //     driveMotorConfig.OpenLoopRamps.VoltageOpenLoopRampPeriod = 0.25; // TODO adjust this later

  //     driveMotorConfig.ClosedLoopRamps.DutyCycleClosedLoopRampPeriod = 0.1; // TODO adjust this
  // later
  //     driveMotorConfig.ClosedLoopRamps.VoltageClosedLoopRampPeriod = 0.1; // TODO Adjust this
  // later

  //     return driveMotorConfig;
  //   }

  public static boolean configureTalonFx(TalonFX motor, TalonFXConfiguration config) {
    if (20 <= motor.getDeviceID() || motor.getDeviceID() <= 27) {
      checkSwerveConfigs(motor, config);
    }

    StatusCode motorStatus = StatusCode.StatusCodeNotInitialized;
    for (int i = 0; i < (RobotBase.isReal() ? 5 : 1); i++) {
      motorStatus = motor.getConfigurator().apply(config);
      if (motorStatus.isOK()) break;
      if (RobotBase.isReal()) Timer.delay(0.02);
    }
    if (!motorStatus.isOK()) {
      var alert =
          new Alert(
              "Could not apply configs to TalonFx ID: "
                  + motor.getDeviceID()
                  + ". Error code: "
                  + motorStatus,
              AlertType.kError);
      alert.set(true);
    } else System.out.println("TalonFX ID: " + motor.getDeviceID() + " - Successfully configured!");

    return motorStatus.isOK();
  }

  private static void checkSwerveConfigs(TalonFX motor, TalonFXConfiguration config) {
    int deviceType = motor.getDeviceID() % 2; // 1 == Turn, 0 == Drive;

    if (deviceType == 0) {
      if (config.CustomParams.CustomParam0 != 2 && config.CustomParams.CustomParam0 != 0)
        throw new IllegalArgumentException(
            "Attempting to configure Drive Motor with Turn Configs!");
    } else if (deviceType == 1) {
      if (config.CustomParams.CustomParam0 != 1 && config.CustomParams.CustomParam0 != 0)
        throw new IllegalArgumentException(
            "Attempting to configure Turn Motor with Drive Configs!");
    }
  }

  public static boolean configureCANCoder(CANcoder cancoder, CANcoderConfiguration config) {
    StatusCode canCoderStatus = StatusCode.StatusCodeNotInitialized;
    for (int i = 0; i < (RobotBase.isReal() ? 5 : 1); i++) {
      canCoderStatus = cancoder.getConfigurator().apply(config);
      if (canCoderStatus.isOK()) break;
      if (RobotBase.isReal()) Timer.delay(0.02);
    }
    if (!canCoderStatus.isOK()) {
      var alert =
          new Alert(
              "Could not apply configs to CANCoder ID: "
                  + cancoder.getDeviceID()
                  + ". Error code: "
                  + canCoderStatus,
              AlertType.kError);
      alert.set(true);
    } else
      System.out.println("CANCoder ID: " + cancoder.getDeviceID() + " - Successfully configured!");
    return canCoderStatus.isOK();
  }
}
