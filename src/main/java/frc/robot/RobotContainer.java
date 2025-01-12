// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.swerve.SwerveRequest;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.commands.Algae.SetAlgaeIntakeSpeed;
import frc.robot.commands.ResetGyro;
import frc.robot.commands.RunCoralOuttake;
import frc.robot.constants.SWERVE;
import frc.robot.constants.USB;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.AlgaeIntake;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.CoralOuttake;
import frc.robot.utils.Telemetry;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  private final CommandSwerveDrivetrain m_swerveDrive = TunerConstants.createDrivetrain();
  private final Telemetry m_telemetry = new Telemetry();
  private final CoralOuttake m_coralOuttake = new CoralOuttake();
  private final AlgaeIntake m_AlgaeIntake = new AlgaeIntake();

  // Replace with CommandPS4Controller or CommandJoystick if needed
  private final Joystick leftJoystick = new Joystick(USB.leftJoystick);
  private final SendableChooser<Command> m_sysidChooser = new SendableChooser<>();
  private final Joystick rightJoystick = new Joystick(USB.rightJoystick);
  private final CommandXboxController m_driverController =
      new CommandXboxController(USB.xBoxController);

  private double MaxSpeed =
      TunerConstants.kSpeedAt12Volts.in(MetersPerSecond); // kSpeedAt12Volts desired top speed
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
    m_swerveDrive.registerTelemetry(m_telemetry::telemeterize);
    
    initSmartDashboard();
    initializeSubSystems();
    
    // Configure the trigger bindings
    configureBindings();
    initAutoChooser();
  }

  private void initSmartDashboard() {
    SmartDashboard.putData("ResetGyro", new ResetGyro(m_swerveDrive));
  }

  private void initializeSubSystems() {
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
    // Add the autonomous chooser to the dashboard
    // SmartDashboard.putData("Auto Mode", m_chooser);
  }

  private void initSmartDashboard() {
    // if (ROBOT.useSysID) initSysidChooser();
    // else initAutoChooser();
    SmartDashboard.putData("ResetGyro", new ResetGyro(m_swerveDrive));
  }

  // private void initSysidChooser(){
  //    SignalLogger.setPath("/media/sda1/");

  //        SysIdUtils.createSwerveDriveRoutines(m_swerveDrive);
  //        SysIdUtils.createSwerveTurnRoutines(m_swerveDrive);

  //        SmartDashboard.putData(
  //       "Start Logging", new InstantCommand(SignalLogger::start).ignoringDisable(true));
  //   SmartDashboard.putData(
  //       "Stop Logging", new InstantCommand(SignalLogger::stop).ignoringDisable(true));
  //   SmartDashboard.putData(
  //       "initDriveSettings",
  //       new InstantCommand(m_swerveDrive::initDriveSysid).ignoringDisable(true));
  //   SmartDashboard.putData(
  //       "initTurnSettings",new
  // InstantCommand(m_swerveDrive::initTurnSysid).ignoringDisable(true));

  //   m_sysidChooser.addOption(
  //       "driveQuasistaticForward",
  //       new SwerveDriveQuasistatic(m_swerveDrive, SysIdRoutine.Direction.kForward));
  //   m_sysidChooser.addOption(
  //       "driveQuasistaticBackwards",
  //       new SwerveDriveQuasistatic(m_swerveDrive, SysIdRoutine.Direction.kReverse));
  //   m_sysidChooser.addOption(
  //       "driveDynamicForward",
  //       new SwerveDriveDynamic(m_swerveDrive, SysIdRoutine.Direction.kForward));
  //   m_sysidChooser.addOption(
  //       "driveDynamicBackward",
  //       new SwerveDriveDynamic(m_swerveDrive, SysIdRoutine.Direction.kReverse));

  //   m_sysidChooser.addOption(
  //       "turnQuasistaticForward",
  //       new SwerveTurnQuasistatic(m_swerveDrive, SysIdRoutine.Direction.kForward));
  //   m_sysidChooser.addOption(
  //       "turnQuasistaticBackwards",
  //       new SwerveTurnQuasistatic(m_swerveDrive, SysIdRoutine.Direction.kReverse));
  //   m_sysidChooser.addOption(
  //       "turnDynamicForward",
  //       new SwerveTurnDynamic(m_swerveDrive, SysIdRoutine.Direction.kForward));
  //   m_sysidChooser.addOption(
  //       "turnDynamicBackward",
  //       new SwerveTurnDynamic(m_swerveDrive, SysIdRoutine.Direction.kReverse));

  // }
  /**
   * Use this method to define your trigger->command mappings. Triggers can be created via the
   * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with an arbitrary
   * predicate, or via the named factories in {@link
   * edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for {@link
   * CommandXboxController Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller
   * PS4} controllers or {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight
   * joysticks}.
   */
  private void configureBindings() {
    //    // Schedule `ExampleCommand` when `exampleCondition` changes to `true`
    //    new Trigger(m_exampleSubsystem::exampleCondition)
    //        .onTrue(new ExampleCommand(m_exampleSubsystem));
    //
    //    // Schedule `exampleMethodCommand` when the Xbox controller's B button is pressed,
    //    // cancelling on release.
    //    m_driverController.b().whileTrue(m_exampleSubsystem.exampleMethodCommand());
    m_driverController.leftBumper().whileTrue(new RunCoralOuttake(m_coralOuttake, 0.15)); // outtake
    m_driverController
        .rightBumper()
        .whileTrue(new RunCoralOuttake(m_coralOuttake, -0.15)); // intake
    m_driverController.x().whileTrue(new SetAlgaeIntakeSpeed(m_AlgaeIntake, 0.5)); // outtake
    m_driverController.y().whileTrue(new SetAlgaeIntakeSpeed(m_AlgaeIntake, -0.5)); // intake
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    return new WaitCommand(0);
  }
}
