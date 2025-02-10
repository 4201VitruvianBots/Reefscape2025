// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.SignalLogger;
import com.ctre.phoenix6.swerve.SwerveRequest;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.commands.RunClimberIntake;
import frc.robot.commands.alphabot.RunAlgaeIntake;
import frc.robot.commands.alphabot.RunCoralOuttake;
import frc.robot.commands.autos.DriveForward;
import frc.robot.commands.autos.TestAuto1;
import frc.robot.commands.ground.GroundPivotControlMode;
import frc.robot.commands.ground.GroundPivotJoystick;
import frc.robot.commands.ground.GroundPivotSetpoint;
import frc.robot.commands.ground.SetGroundIntake;
import frc.robot.commands.swerve.ResetGyro;
import frc.robot.commands.swerve.SwerveCharacterization;
import frc.robot.constants.GROUND.INTAKE.INTAKE_SPEED;
import frc.robot.constants.GROUND.PIVOT.PIVOT_SETPOINT;
import frc.robot.constants.ROBOT;
import frc.robot.constants.SWERVE;
import frc.robot.constants.SWERVE.ROUTINE_TYPE;
import frc.robot.constants.USB;
import frc.robot.generated.V2Constants;
import frc.robot.subsystems.*;
import frc.robot.subsystems.alphabot.*;
import frc.robot.utils.SysIdUtils;
import frc.robot.utils.Telemetry;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  private final CommandSwerveDrivetrain m_swerveDrive;
  private final Telemetry m_telemetry = new Telemetry();
  private final Vision m_vision = new Vision();

  // AlphaBot subsystems
  private CoralOuttake m_coralOuttake;
  private AlgaeIntake m_algaeIntake;

  // V2 subsystems
  //   private Elevator m_elevator;
  //   private EndEffector m_endEffector;
  private ClimberIntake m_climberIntake;
  private Climber m_climber;

  private GroundPivot m_groundPivot;
  private GroundIntake m_groundIntake;

  // Replace with CommandPS4Controller or CommandJoystick if needed
  private final Joystick leftJoystick = new Joystick(USB.leftJoystick);
  private final SendableChooser<Command> m_sysidChooser = new SendableChooser<>();
  private final SendableChooser<Command> m_chooser = new SendableChooser<>();
  private final Joystick rightJoystick = new Joystick(USB.rightJoystick);
  private final CommandXboxController m_driverController =
      new CommandXboxController(USB.xBoxController);

  private double MaxSpeed =
      V2Constants.kSpeedAt12Volts.in(MetersPerSecond); // kSpeedAt12Volts desired top speed
  private double MaxAngularRate =
      RotationsPerSecond.of(SWERVE.kMaxRotationRadiansPerSecond)
          .in(RadiansPerSecond); // 3/4 of a rotation per second max angular velocity

  /* Setting up bindings for necessary control of the swerve drive platform */
  private final SwerveRequest.FieldCentric drive =
      new SwerveRequest.FieldCentric()
          .withDeadband(MaxSpeed * 0.1)
          .withRotationalDeadband(MaxAngularRate * 0.1); // Add a 10% deadband

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    m_swerveDrive = SWERVE.selectedDrivetrain;
    m_swerveDrive.registerTelemetry(m_telemetry::telemeterize);
    m_vision.registerSwerveDrive(m_swerveDrive);
    initSmartDashboard();
    initializeSubSystems();

    // Configure the trigger bindings
    if (ROBOT.robotID.equals(ROBOT.ROBOT_ID.ALPHABOT)) configureAlphaBotBindings();
    else configureV2Bindings();
  }

  private void initializeSubSystems() {
    if (ROBOT.robotID.equals(ROBOT.ROBOT_ID.ALPHABOT)) {
      m_coralOuttake = new CoralOuttake();
      m_algaeIntake = new AlgaeIntake();
    } else {
      //   m_elevator = new Elevator();
      //   m_endEffector = new EndEffector();
      m_climberIntake = new ClimberIntake();
      m_climber = new Climber();
      m_groundIntake = new GroundIntake();
      m_groundPivot = new GroundPivot();

      m_groundPivot.setDefaultCommand(
          new GroundPivotJoystick(
              m_groundPivot, () -> m_driverController.getLeftY())); // For testing purposes
    }

    m_swerveDrive.setDefaultCommand(
        // Drivetrain will execute this command periodically
        m_swerveDrive.applyRequest(
            () ->
                drive
                    .withVelocityX(
                        leftJoystick.getRawAxis(1)
                            * MaxSpeed) // Drive forward with negative Y (forward)
                    .withVelocityY(
                        leftJoystick.getRawAxis(0) * MaxSpeed) // Drive left with negative X (left)
                    .withRotationalRate(
                        rightJoystick.getRawAxis(0)
                            * MaxAngularRate) // Drive counterclockwise with negative X (left)
            ));
  }

  private void initAutoChooser() {
    SmartDashboard.putData("Auto Mode", m_chooser);
    m_chooser.setDefaultOption("Do Nothing", new WaitCommand(0));

    m_chooser.addOption("DriveForward", new DriveForward(m_swerveDrive));
    m_chooser.addOption("TestAuto1", new TestAuto1(m_swerveDrive));
  }

  private void initSmartDashboard() {
    if (ROBOT.useSysID) initSysidChooser();
    else initAutoChooser();
    SmartDashboard.putData("ResetGyro", new ResetGyro(m_swerveDrive));
  }

  private void initSysidChooser() {
    SignalLogger.setPath("/media/sda1/");

    SysIdUtils.createSwerveDriveRoutines(m_swerveDrive);
    SysIdUtils.createSwerveTurnRoutines(m_swerveDrive);

    SmartDashboard.putData(
        "Start Logging", new InstantCommand(SignalLogger::start).ignoringDisable(true));
    SmartDashboard.putData(
        "Stop Logging", new InstantCommand(SignalLogger::stop).ignoringDisable(true));
    SmartDashboard.putData(
        "initDriveSettings",
        new InstantCommand(m_swerveDrive::initDriveSysid).ignoringDisable(true));

    m_sysidChooser.addOption(
        "driveQuasistaticForward",
        new SwerveCharacterization(
            m_swerveDrive, SysIdRoutine.Direction.kForward, ROUTINE_TYPE.DRIVE_QUASISTATIC));
    m_sysidChooser.addOption(
        "driveQuasistaticBackwards",
        new SwerveCharacterization(
            m_swerveDrive, SysIdRoutine.Direction.kReverse, ROUTINE_TYPE.DRIVE_QUASISTATIC));
    m_sysidChooser.addOption(
        "driveDynamicForward",
        new SwerveCharacterization(
            m_swerveDrive, SysIdRoutine.Direction.kForward, ROUTINE_TYPE.DRIVE_DYNAMIC));
    m_sysidChooser.addOption(
        "driveDynamicBackward",
        new SwerveCharacterization(
            m_swerveDrive, SysIdRoutine.Direction.kReverse, ROUTINE_TYPE.DRIVE_DYNAMIC));

    m_sysidChooser.addOption(
        "turnQuasistaticForward",
        new SwerveCharacterization(
            m_swerveDrive, SysIdRoutine.Direction.kForward, ROUTINE_TYPE.TURN_QUASISTATIC));
    m_sysidChooser.addOption(
        "turnQuasistaticBackwards",
        new SwerveCharacterization(
            m_swerveDrive, SysIdRoutine.Direction.kReverse, ROUTINE_TYPE.TURN_QUASISTATIC));
    m_sysidChooser.addOption(
        "turnDynamicForward",
        new SwerveCharacterization(
            m_swerveDrive, SysIdRoutine.Direction.kForward, ROUTINE_TYPE.TURN_DYNAMIC));
    m_sysidChooser.addOption(
        "turnDynamicBackward",
        new SwerveCharacterization(
            m_swerveDrive, SysIdRoutine.Direction.kReverse, ROUTINE_TYPE.TURN_DYNAMIC));
  }

  private void configureAlphaBotBindings() {
    m_driverController.leftBumper().whileTrue(new RunCoralOuttake(m_coralOuttake, 0.15)); // outtake
    m_driverController
        .rightBumper()
        .whileTrue(new RunCoralOuttake(m_coralOuttake, -0.15)); // intake
    m_driverController.x().whileTrue(new RunAlgaeIntake(m_algaeIntake, 0.5)); // outtake
    m_driverController.y().whileTrue(new RunAlgaeIntake(m_algaeIntake, -0.5)); // intake
  }

  private void configureV2Bindings() {
    // m_driverController
    //     .leftTrigger()
    //     .whileTrue(new RunEndEffectorIntake(m_endEffector, 0.4414)); // intake
    // m_driverController.povLeft().whileTrue(new RunClimberIntake(m_climberIntake, 0.25));
    // m_driverController.povRight().onTrue(new SetClimberSetpoint(m_climber,
    // CLIMBER_SETPOINT.CLIMB));

    m_driverController.back().onTrue(new GroundPivotControlMode(m_groundPivot));

    m_driverController
        .leftTrigger()
        .whileTrue(new SetGroundIntake(m_groundIntake, INTAKE_SPEED.HOLDING_CORAL));
    m_driverController
        .rightTrigger()
        .whileTrue(new SetGroundIntake(m_groundIntake, INTAKE_SPEED.OUTTAKE_TO_END_EFFECTOR));
    m_driverController
        .leftBumper()
        .whileTrue(new SetGroundIntake(m_groundIntake, INTAKE_SPEED.INTAKING));
    m_driverController
        .rightBumper()
        .whileTrue(new SetGroundIntake(m_groundIntake, INTAKE_SPEED.OUTTAKING));

    m_driverController.a().whileTrue(new GroundPivotSetpoint(m_groundPivot, PIVOT_SETPOINT.STOWED));
    m_driverController.b().whileTrue(new GroundPivotSetpoint(m_groundPivot, PIVOT_SETPOINT.ALGAE));
    m_driverController
        .x()
        .whileTrue(new GroundPivotSetpoint(m_groundPivot, PIVOT_SETPOINT.GROUND_INTAKE));

    m_driverController.povUp().whileTrue(new RunClimberIntake(m_climberIntake, 0.25));
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    return m_chooser.getSelected();
  }

  public void autonomousInit() {
    m_groundPivot.autonomousInit();
  }

  public void teleopInit() {
    m_groundPivot.teleopInit();
  }

  public void testInit() {
    try {
      if (ROBOT.robotID.equals(ROBOT.ROBOT_ID.ALPHABOT)) m_coralOuttake.testInit();
      else {
        m_groundPivot.testInit();
        m_groundIntake.testInit();
      }
    } catch (Exception e) {
      e.printStackTrace();
    }
  }

  public void testPeriodic() {
    try {
      if (ROBOT.robotID.equals(ROBOT.ROBOT_ID.ALPHABOT)) m_coralOuttake.testPeriodic();
      else {
        m_groundPivot.testPeriodic();
        m_groundIntake.testPeriodic();
      }
    } catch (Exception e) {
      e.printStackTrace();
    }
  }
}
