// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.SignalLogger;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.swerve.SwerveRequest;
import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.epilogue.NotLogged;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.commands.RunClimberIntake;
import frc.robot.commands.RunEndEffectorIntake;
import frc.robot.commands.SetHopperIntake;
import frc.robot.commands.ToggleGamePiece;
import frc.robot.commands.alphabot.RunAlgaeIntake;
import frc.robot.commands.alphabot.RunCoralOuttake;
import frc.robot.commands.autos.DriveForward;
import frc.robot.commands.autos.TestAuto1;
import frc.robot.commands.climber.SetClimberSetpoint;
import frc.robot.commands.elevator.RunElevatorJoystick;
import frc.robot.commands.elevator.SetElevatorSetpoint;
import frc.robot.commands.endEffector.EndEffectorJoystick;
import frc.robot.commands.endEffector.EndEffectorSetpoint;
import frc.robot.commands.swerve.ResetGyro;
import frc.robot.commands.swerve.SwerveCharacterization;
import frc.robot.constants.CLIMBER.CLIMBER_SETPOINT;
import frc.robot.constants.FIELD;
import frc.robot.constants.HOPPERINTAKE;
import frc.robot.constants.ROBOT;
import frc.robot.constants.ROBOT.SUPERSTRUCTURE_STATES;
import frc.robot.constants.SWERVE;
import frc.robot.constants.SWERVE.ROUTINE_TYPE;
import frc.robot.constants.USB;
import frc.robot.generated.AlphaBotConstants;
import frc.robot.generated.V2Constants;
import frc.robot.subsystems.*;
import frc.robot.subsystems.alphabot.*;
import frc.robot.utils.Robot2d;
import frc.robot.utils.SysIdUtils;
import frc.robot.utils.Telemetry;
import java.util.Arrays;
import org.team4201.codex.simulation.FieldSim;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
@Logged
public class RobotContainer {
  private CommandSwerveDrivetrain m_swerveDrive;

  @Logged(name = "Controls")
  private final Controls m_controls = new Controls();

  private final Telemetry m_telemetry = new Telemetry();
  private final FieldSim m_fieldSim = new FieldSim();
  private final Vision m_vision = new Vision();

  // AlphaBot subsystems
  private CoralOuttake m_coralOuttake;
  private AlgaeIntake m_algaeIntake;

  // V2 subsystems
  private Climber m_climber;
  private ClimberIntake m_climberIntake;
  private Elevator m_elevator;

  @Logged(name = "EndEffector")
  private EndEffector m_endEffector;

  @Logged(name = "EndEffectorPivot")
  private EndEffectorPivot m_endEffectorPivot;

  private HopperIntake m_hopperIntake;

  private final Robot2d m_robot2d = new Robot2d();

  private ROBOT.GAME_PIECE m_selectedGamePiece = ROBOT.GAME_PIECE.CORAL;

  // Replace with CommandPS4Controller or CommandJoystick if needed
  private final Joystick leftJoystick = new Joystick(USB.leftJoystick);
  @NotLogged private final SendableChooser<Command> m_sysidChooser = new SendableChooser<>();

  @Logged(name = "AutoChooser")
  private final SendableChooser<Command> m_chooser = new SendableChooser<>();

  private final Joystick rightJoystick = new Joystick(USB.rightJoystick);
  private final CommandXboxController m_driverController =
      new CommandXboxController(USB.xBoxController);

  @NotLogged
  private double MaxSpeed =
      AlphaBotConstants.kSpeedAt12Volts.in(MetersPerSecond); // kSpeedAt12Volts desired top speed

  @NotLogged
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
    initializeSubSystems();
    initSmartDashboard();

    // Configure the trigger bindings
    if (ROBOT.robotID.equals(ROBOT.ROBOT_ID.ALPHABOT)) configureAlphaBotBindings();
    else configureV2Bindings();

    // Only keep joystick warnings when FMS is attached
    if (!DriverStation.isFMSAttached()) {
      DriverStation.silenceJoystickConnectionWarning(true);
    }
  }

  private void initializeSubSystems() {
    // Initialize Subsystem classes
    if (ROBOT.robotID.equals(ROBOT.ROBOT_ID.V2)) {
      m_swerveDrive = V2Constants.createDrivetrain();
      m_elevator = new Elevator();
      m_endEffector = new EndEffector();
      m_endEffectorPivot = new EndEffectorPivot();
      m_climberIntake = new ClimberIntake();
      m_climber = new Climber();
      m_hopperIntake = new HopperIntake();
    } else if (ROBOT.robotID.equals(ROBOT.ROBOT_ID.ALPHABOT)) {
      m_swerveDrive = AlphaBotConstants.createDrivetrain();
      // m_coralOuttake = new CoralOuttake();
      // m_algaeIntake = new AlgaeIntake();
    } else if (ROBOT.robotID.equals(ROBOT.ROBOT_ID.SIM)) {
      m_swerveDrive = V2Constants.createDrivetrain();
      m_elevator = new Elevator();
      m_endEffector = new EndEffector();
      m_endEffectorPivot = new EndEffectorPivot();
      m_climberIntake = new ClimberIntake();
      m_climber = new Climber();
      m_hopperIntake = new HopperIntake();
    } else {
      // Most likely, the code will crash later on if you get here
      DriverStation.reportError(
          "[RobotContainer] Unhandled initSubsystem for RobotID " + ROBOT.robotID.getName(), false);
    }
    // Register subsystems for cross-subsystem communication
    m_swerveDrive.registerTelemetry(m_telemetry::telemeterize);
    m_vision.registerSwerveDrive(m_swerveDrive);

    m_telemetry.registerFieldSim(m_fieldSim);

    // TODO: Enable this when subsystems are implemented
    m_robot2d.registerSubsystem(m_elevator);
    m_robot2d.registerSubsystem(m_endEffectorPivot);
    m_robot2d.registerSubsystem(m_endEffector);

    // Set Subsystem DefaultCommands
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
    if (m_elevator != null) {
      m_elevator.setDefaultCommand(
          new RunElevatorJoystick(m_elevator, () -> -m_driverController.getLeftY()));
    }
    if (m_endEffectorPivot != null) {
      m_endEffectorPivot.setDefaultCommand(
          new EndEffectorJoystick(m_endEffectorPivot, () -> -m_driverController.getRightY()));
    }
  }

  private void initAutoChooser() {
    SmartDashboard.putData("Auto Mode", m_chooser);
    m_chooser.setDefaultOption("Do Nothing", new WaitCommand(0));

    m_chooser.addOption("DriveForward", new DriveForward(m_swerveDrive, m_fieldSim));
    m_chooser.addOption("TestAuto1", new TestAuto1(m_swerveDrive, m_fieldSim));
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
    if (m_coralOuttake != null) {
      // TODO: Make speeds into enum setpoints
      m_driverController
          .leftBumper()
          .whileTrue(new RunCoralOuttake(m_coralOuttake, 0.15)); // outtake
      m_driverController
          .rightBumper()
          .whileTrue(new RunCoralOuttake(m_coralOuttake, -0.15)); // intake
    }

    if (m_endEffectorPivot != null) {
      //   m_driverController
      //       .a()
      //       .whileTrue(new EndEffectorSetpoint(m_endEffectorPivot, PIVOT_SETPOINT.L3_L2));
    }
    if (m_endEffector != null) {
      //   m_driverController
      //       .leftTrigger()
      //       .whileTrue(new RunEndEffectorIntake(m_endEffector, 0.4414)); // intake
      //   m_driverController
      //       .rightTrigger()
      //       .whileTrue(new RunEndEffectorIntake(m_endEffector, -0.4414)); // outtake?
    }

    if (m_algaeIntake != null) {
      // TODO: Make speeds into enum setpoints
      m_driverController.x().whileTrue(new RunAlgaeIntake(m_algaeIntake, 0.5)); // outtake
      m_driverController.y().whileTrue(new RunAlgaeIntake(m_algaeIntake, -0.5)); // intake
    }
  }

  private void configureV2Bindings() {
    ParallelRaceGroup stowAll =
        new ParallelCommandGroup(
                new EndEffectorSetpoint(
                    m_endEffectorPivot, SUPERSTRUCTURE_STATES.STOWED, () -> m_selectedGamePiece),
                new SetElevatorSetpoint(
                    m_elevator, SUPERSTRUCTURE_STATES.STOWED, () -> m_selectedGamePiece))
            .withTimeout(1);

    ParallelRaceGroup stowAllDelayed =
        new SequentialCommandGroup(
                new EndEffectorSetpoint(
                        m_endEffectorPivot, SUPERSTRUCTURE_STATES.STOWED, () -> m_selectedGamePiece)
                    .withTimeout(0.7),
                new SetElevatorSetpoint(
                    m_elevator, SUPERSTRUCTURE_STATES.STOWED, () -> m_selectedGamePiece))
            .withTimeout(1);

    // Algae Toggle
    m_driverController
        .leftBumper()
        .onTrue(new ToggleGamePiece(() -> m_selectedGamePiece, gp -> m_selectedGamePiece = gp));

    if (m_elevator != null && m_endEffectorPivot != null) {
      m_driverController
          .a()
          .whileTrue(
              new ParallelCommandGroup(
                  new SetElevatorSetpoint(
                      m_elevator, SUPERSTRUCTURE_STATES.L2, () -> m_selectedGamePiece),
                  new EndEffectorSetpoint(
                      m_endEffectorPivot, SUPERSTRUCTURE_STATES.L2, () -> m_selectedGamePiece)))
          .onFalse(stowAll);
      m_driverController
          .x()
          .whileTrue(
              new ParallelCommandGroup(
                  new SetElevatorSetpoint(
                      m_elevator, SUPERSTRUCTURE_STATES.L1, () -> m_selectedGamePiece),
                  new EndEffectorSetpoint(
                      m_endEffectorPivot, SUPERSTRUCTURE_STATES.L1, () -> m_selectedGamePiece)))
          .onFalse(stowAll);
      m_driverController
          .y()
          .whileTrue(
              new SequentialCommandGroup(
                  new SetElevatorSetpoint(
                          m_elevator, SUPERSTRUCTURE_STATES.L4, () -> m_selectedGamePiece)
                      .withTimeout(0.7),
                  new EndEffectorSetpoint(
                      m_endEffectorPivot, SUPERSTRUCTURE_STATES.L4, () -> m_selectedGamePiece)))
          .onFalse(stowAllDelayed);
      m_driverController
          .b()
          .whileTrue(
              new SequentialCommandGroup(
                  new SetElevatorSetpoint(
                          m_elevator, SUPERSTRUCTURE_STATES.L3, () -> m_selectedGamePiece)
                      .withTimeout(0.5),
                  new EndEffectorSetpoint(
                      m_endEffectorPivot, SUPERSTRUCTURE_STATES.L3, () -> m_selectedGamePiece)))
          .onFalse(stowAll);
      m_driverController.povLeft().whileTrue(new RunClimberIntake(m_climberIntake, 0.25));
    }

    // Ground intake on left trigger, TODO: implement
    // Ground intake algae on povDown, TODO: implement

    // Ready hopper
    if (m_hopperIntake != null
        && m_endEffectorPivot != null
        && m_endEffector != null
        && m_elevator != null) {
      m_driverController
          .povUp()
          .whileTrue(
              new ParallelCommandGroup(
                  new SetHopperIntake(m_hopperIntake, HOPPERINTAKE.INTAKE_SPEED.INTAKING),
                  new EndEffectorSetpoint(
                      m_endEffectorPivot,
                      SUPERSTRUCTURE_STATES.HOPPER_INTAKE,
                      () -> m_selectedGamePiece),
                  new RunEndEffectorIntake(
                      m_endEffector,
                      false,
                      () -> m_selectedGamePiece), // TODO Gavin fix this part pls
                  new SetElevatorSetpoint(
                      m_elevator, SUPERSTRUCTURE_STATES.HOPPER_INTAKE, () -> m_selectedGamePiece)))
          .onFalse(stowAll);
    }

    if (m_endEffector != null) {
      // Score Coral / Algae Intake
      m_driverController
          .rightTrigger()
          .whileTrue(new RunEndEffectorIntake(m_endEffector, true, () -> m_selectedGamePiece));
      // Coral Reverse / Algae Outtake
      m_driverController
          .rightBumper()
          .whileTrue(new RunEndEffectorIntake(m_endEffector, false, () -> m_selectedGamePiece));
    }

    if (m_climber != null) {
      m_driverController
          .povRight()
          .onTrue(new SetClimberSetpoint(m_climber, CLIMBER_SETPOINT.CLIMB));
    }
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    return m_chooser.getSelected();
  }

  public void disabledInit() {
    if (!DriverStation.isFMSAttached()) {
      m_swerveDrive.setNeutralMode(SWERVE.MOTOR_TYPE.ALL, NeutralModeValue.Coast);
    }
  }

  public void autonomousInit() {
    m_swerveDrive.setNeutralMode(SWERVE.MOTOR_TYPE.ALL, NeutralModeValue.Brake);
  }

  public void teleopInit() {
    m_swerveDrive.setNeutralMode(SWERVE.MOTOR_TYPE.ALL, NeutralModeValue.Brake);
    m_elevator.teleopInit();
  }

  public void testInit() {
    try {
      if (ROBOT.robotID.equals(ROBOT.ROBOT_ID.ALPHABOT)) m_coralOuttake.testInit();
    } catch (Exception e) {
      e.printStackTrace();
    }
  }

  public void testPeriodic() {
    try {
      if (ROBOT.robotID.equals(ROBOT.ROBOT_ID.ALPHABOT)) m_coralOuttake.testPeriodic();
    } catch (Exception e) {
      e.printStackTrace();
    }
  }

  public void simulationInit() {
    //    m_fieldSim.addStaticPoses("ReefBranches", FIELD.REEF_BRANCHES.getAllPose2d());
    m_fieldSim.initializePoses("Red Branches", FIELD.RED_BRANCHES);
    m_fieldSim.initializePoses("Blue Branches", FIELD.BLUE_BRANCHES);
    m_fieldSim.initializePoses("Red Zones", FIELD.RED_ZONES);
    m_fieldSim.initializePoses("Blue Zones", FIELD.BLUE_ZONES);

    //    m_fieldSim.initializePoses(
    //        "RED_REEF_NEAR_LEFT AprilTag", FIELD.APRIL_TAG.RED_REEF_NEAR_LEFT.getPose2d());
    //    m_fieldSim.initializePoses(
    //        "RED_REEF_NEAR_LEFT_LEFT", FIELD.REEF_BRANCHES.RED_REEF_NEAR_LEFT_LEFT.getPose2d());
    //    m_fieldSim.initializePoses(
    //        "RED_REEF_NEAR_LEFT_RIGHT", FIELD.REEF_BRANCHES.RED_REEF_NEAR_LEFT_RIGHT.getPose2d());

    //    m_fieldSim.addStaticPoses("RED_REEF_NEAR_LEFT_LEFT_ZONE",
    // FIELD.ZONES.RED_REEF_NEAR_LEFT_LEFT.getZone());
    //    m_fieldSim.addStaticPoses("RED_REEF_NEAR_LEFT_RIGHT_ZONE",
    // FIELD.ZONES.RED_REEF_NEAR_LEFT_RIGHT.getZone());
    //    m_fieldSim.addStaticPoses("BLUE_REEF_NEAR_LEFT_LEFT_ZONE",
    // FIELD.ZONES.BLUE_REEF_NEAR_LEFT_LEFT.getZone());
    //    m_fieldSim.addStaticPoses("BLUE_REEF_NEAR_LEFT_RIGHT_ZONE",
    // FIELD.ZONES.BLUE_REEF_NEAR_LEFT_RIGHT.getZone());

    //    m_fieldSim.addStaticPoses("RED_REEF_NEAR_CENTER AprilTag",
    // FIELD.APRIL_TAG.RED_REEF_NEAR_CENTER.getPose2d());
    //    m_fieldSim.addStaticPoses("RED_REEF_NEAR_CENTER_LEFT",
    // FIELD.REEF_BRANCHES.RED_REEF_NEAR_CENTER_LEFT.getPose2d());
    //    m_fieldSim.addStaticPoses("RED_REEF_NEAR_CENTER_RIGHT",
    // FIELD.REEF_BRANCHES.RED_REEF_NEAR_CENTER_RIGHT.getPose2d());
  }

  public void simulationPeriodic() {
    DriverStation.getAlliance()
        .ifPresent(
            a -> {
              Pose2d[] robotToBranch = {m_swerveDrive.getState().Pose, new Pose2d()};
              switch (a) {
                case Red ->
                    robotToBranch[1] = robotToBranch[0].nearest(Arrays.asList(FIELD.RED_BRANCHES));
                case Blue ->
                    robotToBranch[1] = robotToBranch[0].nearest(Arrays.asList(FIELD.BLUE_BRANCHES));
              }
              m_fieldSim.addPoses("LineToNearestBranch", robotToBranch);
            });
  }

  public void robotPeriodic() {
    m_robot2d.updateRobot2d();
    // m_questNav.periodic();
  }
}
