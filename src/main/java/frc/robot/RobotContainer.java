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
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.*;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.commands.RunHopperIntake;
import frc.robot.commands.ToggleGamePiece;
import frc.robot.commands.alphabot.RunAlgaeIntake;
import frc.robot.commands.alphabot.RunCoralOuttake;
import frc.robot.commands.autos.*;
import frc.robot.commands.climber.RunClimberVoltage;
import frc.robot.commands.climber.RunClimberVoltageJoystick;
import frc.robot.commands.elevator.SetElevatorSetpoint;
import frc.robot.commands.endEffector.EndEffectorJoystick;
import frc.robot.commands.endEffector.EndEffectorSetpoint;
import frc.robot.commands.endEffector.RunEndEffectorIntake;
import frc.robot.commands.swerve.ResetGyro;
import frc.robot.commands.swerve.SetTrackingState;
import frc.robot.commands.swerve.SwerveCharacterization;
import frc.robot.constants.ELEVATOR.ELEVATOR_SETPOINT;
import frc.robot.constants.ENDEFFECTOR.PIVOT.PIVOT_SETPOINT;
import frc.robot.constants.ENDEFFECTOR.ROLLERS.ROLLER_SPEED;
import frc.robot.constants.FIELD;
import frc.robot.constants.HOPPERINTAKE;
import frc.robot.constants.ROBOT;
import frc.robot.constants.SWERVE;
import frc.robot.constants.SWERVE.ROUTINE_TYPE;
import frc.robot.constants.USB;
import frc.robot.constants.VISION.TRACKING_STATE;
import frc.robot.generated.AlphaBotConstants;
import frc.robot.generated.V2Constants;
import frc.robot.subsystems.*;
import frc.robot.subsystems.alphabot.AlgaeIntake;
import frc.robot.subsystems.alphabot.CoralOuttake;
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
@Logged(name = "RobotContainer", importance = Logged.Importance.CRITICAL)
public class RobotContainer {
  @NotLogged private CommandSwerveDrivetrain m_swerveDrive;

  @Logged(name = "Controls", importance = Logged.Importance.INFO)
  private final Controls m_controls = new Controls();

  @NotLogged private final Telemetry m_telemetry = new Telemetry();
  private final FieldSim m_fieldSim = new FieldSim();
  private final Vision m_vision = new Vision();

  // AlphaBot subsystems
  private CoralOuttake m_coralOuttake;
  private AlgaeIntake m_algaeIntake;

  // V2 subsystems
  @Logged(name = "Climber", importance = Logged.Importance.INFO)
  private Climber m_climber;

  @Logged(name = "Elevator", importance = Logged.Importance.INFO)
  private Elevator m_elevator;

  @Logged(name = "EndEffector", importance = Logged.Importance.INFO)
  private EndEffector m_endEffector;

  @Logged(name = "EndEffectorPivot", importance = Logged.Importance.INFO)
  private EndEffectorPivot m_endEffectorPivot;

  @Logged(name = "HopperIntake", importance = Logged.Importance.INFO)
  private HopperIntake m_hopperIntake;

  @NotLogged private final Robot2d m_robot2d = new Robot2d();
  private Pose2d nearestBranchPose = Pose2d.kZero;
  private final Pose2d[] robotToBranch = {Pose2d.kZero, Pose2d.kZero};

  @Logged(name = "Selected Game Piece", importance = Logged.Importance.CRITICAL)
  private ROBOT.GAME_PIECE m_selectedGamePiece = ROBOT.GAME_PIECE.CORAL;

  // Replace with CommandPS4Controller or CommandJoystick if needed
  @Logged(name = "AutoChooser")
  private final SendableChooser<Command> m_chooser = new SendableChooser<>();

  @NotLogged private final SendableChooser<Command> m_sysidChooser = new SendableChooser<>();

  @NotLogged private final Joystick leftJoystick = new Joystick(USB.leftJoystick);
  @NotLogged private final Joystick rightJoystick = new Joystick(USB.rightJoystick);

  @NotLogged
  private final CommandXboxController m_driverController =
      new CommandXboxController(USB.xBoxController);

  @NotLogged private double MaxSpeed;

  @NotLogged
  private final double MaxAngularRate =
      RotationsPerSecond.of(SWERVE.kMaxRotationRadiansPerSecond)
          .in(RadiansPerSecond); // 3/4 of a rotation per second max angular velocity

  /* Setting up bindings for necessary control of the swerve drive platform */
  @NotLogged
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
      MaxSpeed =
          V2Constants.kSpeedAt12Volts.in(MetersPerSecond); // kSpeedAt12Volts desired top speed
      m_swerveDrive = V2Constants.createDrivetrain();
      m_elevator = new Elevator();
      m_endEffector = new EndEffector();
      m_endEffectorPivot = new EndEffectorPivot();
      m_climber = new Climber();
      m_hopperIntake = new HopperIntake();
    } else if (ROBOT.robotID.equals(ROBOT.ROBOT_ID.ALPHABOT)) {
      MaxSpeed =
          AlphaBotConstants.kSpeedAt12Volts.in(
              MetersPerSecond); // kSpeedAt12Volts desired top speed
      m_swerveDrive = AlphaBotConstants.createDrivetrain();
      // m_coralOuttake = new CoralOuttake();
      // m_algaeIntake = new AlgaeIntake();
    } else if (ROBOT.robotID.equals(ROBOT.ROBOT_ID.SIM)) {
      MaxSpeed =
          V2Constants.kSpeedAt12Volts.in(MetersPerSecond); // kSpeedAt12Volts desired top speed
      m_swerveDrive = V2Constants.createDrivetrain();
      m_elevator = new Elevator();
      m_endEffector = new EndEffector();
      m_endEffectorPivot = new EndEffectorPivot();
      m_climber = new Climber();
      m_hopperIntake = new HopperIntake();
    } else {
      // Most likely, the code will crash later on if you get here, so send an error message
      DriverStation.reportError(
          "[RobotContainer] Unhandled initSubsystem for RobotID " + ROBOT.robotID.getName(), false);
    }
    // Register subsystems for cross-subsystem communication
    m_swerveDrive.registerTelemetry(m_telemetry::telemeterize);
    m_vision.registerSwerveDrive(m_swerveDrive);

    m_telemetry.registerFieldSim(m_fieldSim);

    m_robot2d.registerSubsystem(m_elevator);
    m_robot2d.registerSubsystem(m_endEffectorPivot);
    m_robot2d.registerSubsystem(m_endEffector);

    // Set Subsystem DefaultCommands
    m_swerveDrive.setDefaultCommand(
        // Drivetrain will execute this command periodically
        m_swerveDrive.applyRequest(
            () -> {
              var rotationRate = rightJoystick.getRawAxis(0) * MaxAngularRate;
              // if heading target
              if (m_swerveDrive.isTrackingState()) {
                rotationRate = m_swerveDrive.calculateRotationToTarget();
              }
              drive
                  .withVelocityX(
                      leftJoystick.getRawAxis(1)
                          * MaxSpeed) // Drive forward with negative Y (forward)
                  .withVelocityY(
                      leftJoystick.getRawAxis(0) * MaxSpeed) // Drive left with negative X (left)
                  .withRotationalRate(
                      rotationRate); // Drive counterclockwise with negative X (left)
              return drive;
            }));
    // if (m_elevator != null) {
    //   m_elevator.setDefaultCommand(
    //       new RunElevatorJoystick(m_elevator, () -> -m_driverController.getLeftY())); // Elevator
    // open loop control
    // }
    if (m_climber != null) {
      m_climber.setDefaultCommand(
          new RunClimberVoltageJoystick(m_climber, () -> -m_driverController.getLeftY()));
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
    m_chooser.addOption(
        "OnePiece",
        new OnePiece(m_swerveDrive, m_fieldSim, m_elevator, m_endEffectorPivot, m_endEffector));

    m_chooser.addOption(
        "HopperTest",
        new TestHopperAuto(
            m_swerveDrive,
            m_fieldSim,
            m_elevator,
            m_endEffector,
            m_endEffectorPivot,
            m_hopperIntake));

    m_chooser.addOption(
        "TwoPiece",
        new TwoPiece(
            m_swerveDrive,
            m_fieldSim,
            m_elevator,
            m_endEffectorPivot,
            m_endEffector,
            m_hopperIntake));

    m_chooser.addOption(
        "OnePieceLeft",
        new OnePieceLeft(
            m_swerveDrive,
            m_fieldSim,
            m_elevator,
            m_endEffectorPivot,
            m_endEffector,
            m_hopperIntake));

    // m_chooser.addOption(
    //     "OnePieceRight",
    //     new OnePieceRight(
    //         m_swerveDrive,
    //         m_fieldSim,
    //         m_elevator,
    //         m_endEffectorPivot,
    //         m_endEffector,
    //         m_hopperIntake));

    m_chooser.addOption(
        "ThreePiece",
        new ThreePiece(
            m_swerveDrive,
            m_fieldSim,
            m_elevator,
            m_endEffectorPivot,
            m_endEffector,
            m_hopperIntake));
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

  private ParallelCommandGroup moveSuperStructure(
      ELEVATOR_SETPOINT elevatorSetpoint, PIVOT_SETPOINT pivotSetpoint) {
    return new ParallelCommandGroup(
        new SetElevatorSetpoint(m_elevator, elevatorSetpoint),
        new EndEffectorSetpoint(m_endEffectorPivot, pivotSetpoint));
  }

  private SequentialCommandGroup moveSuperStructureDelayed(
      ELEVATOR_SETPOINT elevatorSetpoint, PIVOT_SETPOINT pivotSetpoint) {
    return new SequentialCommandGroup(
        new SetElevatorSetpoint(m_elevator, elevatorSetpoint).withTimeout(0.4),
        new EndEffectorSetpoint(m_endEffectorPivot, pivotSetpoint));
  }

  private void configureV2Bindings() {
    Trigger targetTrackingButton = new Trigger(() -> rightJoystick.getRawButton(2));
    targetTrackingButton.whileTrue(new SetTrackingState(m_swerveDrive, TRACKING_STATE.BRANCH));

    // Algae Toggle
    m_driverController
        .leftBumper()
        .onTrue(new ToggleGamePiece(() -> m_selectedGamePiece, gp -> m_selectedGamePiece = gp));

    if (m_elevator != null && m_endEffectorPivot != null) {
      m_driverController
          .a()
          .whileTrue(
              new ConditionalCommand(
                  moveSuperStructure(
                      ELEVATOR_SETPOINT.ALGAE_REEF_INTAKE_LOWER,
                      PIVOT_SETPOINT.INTAKE_ALGAE_LOW), // Algae L2
                  moveSuperStructure(ELEVATOR_SETPOINT.LEVEL_2, PIVOT_SETPOINT.L3_L2), // Coral L2
                  () -> m_selectedGamePiece == ROBOT.GAME_PIECE.ALGAE))
          .onFalse(
              new ConditionalCommand(
                  moveSuperStructure(
                          ELEVATOR_SETPOINT.PROCESSOR, PIVOT_SETPOINT.OUTTAKE_ALGAE_PROCESSOR)
                      .withTimeout(1),
                  moveSuperStructure(ELEVATOR_SETPOINT.START_POSITION, PIVOT_SETPOINT.STOWED)
                      .withTimeout(1),
                  () -> m_selectedGamePiece == ROBOT.GAME_PIECE.ALGAE));
      m_driverController
          .x()
          .whileTrue(
              new ConditionalCommand(
                  moveSuperStructure(
                      ELEVATOR_SETPOINT.PROCESSOR,
                      PIVOT_SETPOINT.OUTTAKE_ALGAE_PROCESSOR), // Algae L1
                  moveSuperStructure(
                      ELEVATOR_SETPOINT.START_POSITION, PIVOT_SETPOINT.STOWED), // Coral L1
                  () -> m_selectedGamePiece == ROBOT.GAME_PIECE.ALGAE))
          .onFalse(
              new ConditionalCommand(
                  moveSuperStructure(
                          ELEVATOR_SETPOINT.PROCESSOR, PIVOT_SETPOINT.OUTTAKE_ALGAE_PROCESSOR)
                      .withTimeout(1),
                  moveSuperStructure(ELEVATOR_SETPOINT.START_POSITION, PIVOT_SETPOINT.STOWED)
                      .withTimeout(1),
                  () -> m_selectedGamePiece == ROBOT.GAME_PIECE.ALGAE));
      m_driverController
          .y()
          .whileTrue(
              new ConditionalCommand(
                  moveSuperStructureDelayed(
                      ELEVATOR_SETPOINT.LEVEL_4, PIVOT_SETPOINT.BARGE), // Algae L4
                  moveSuperStructureDelayed(
                      ELEVATOR_SETPOINT.LEVEL_4, PIVOT_SETPOINT.L4), // Coral L4
                  () -> m_selectedGamePiece == ROBOT.GAME_PIECE.ALGAE))
          .onFalse(
              new ConditionalCommand(
                  new SequentialCommandGroup(
                          new EndEffectorSetpoint(m_endEffectorPivot, PIVOT_SETPOINT.STOWED)
                              .withTimeout(0.7),
                          new SetElevatorSetpoint(m_elevator, ELEVATOR_SETPOINT.START_POSITION))
                      .withTimeout(1),
                  new SequentialCommandGroup(
                          new EndEffectorSetpoint(m_endEffectorPivot, PIVOT_SETPOINT.STOWED)
                              .withTimeout(0.7),
                          new SetElevatorSetpoint(m_elevator, ELEVATOR_SETPOINT.START_POSITION))
                      .withTimeout(1),
                  () -> m_selectedGamePiece == ROBOT.GAME_PIECE.ALGAE));
      m_driverController
          .b()
          .whileTrue(
              new ConditionalCommand(
                  moveSuperStructure(
                      ELEVATOR_SETPOINT.ALGAE_REEF_INTAKE_UPPER,
                      PIVOT_SETPOINT.INTAKE_ALGAE_HIGH), // Algae L3
                  moveSuperStructure(ELEVATOR_SETPOINT.LEVEL_3, PIVOT_SETPOINT.L3_L2), // Coral L3
                  () -> m_selectedGamePiece == ROBOT.GAME_PIECE.ALGAE))
          .onFalse(
              new ConditionalCommand(
                  moveSuperStructure(
                          ELEVATOR_SETPOINT.PROCESSOR, PIVOT_SETPOINT.OUTTAKE_ALGAE_PROCESSOR)
                      .withTimeout(1),
                  moveSuperStructure(ELEVATOR_SETPOINT.START_POSITION, PIVOT_SETPOINT.STOWED)
                      .withTimeout(1),
                  () -> m_selectedGamePiece == ROBOT.GAME_PIECE.ALGAE));
    }

    // Ground intake on left trigger, TODO: implement
    // Ground intake algae on povDown, TODO: implement

    if (m_hopperIntake != null
        && m_endEffectorPivot != null
        && m_endEffector != null
        && m_elevator != null) {
      // Ready hopper
      m_driverController
          .povUp()
          .whileTrue(
              new ParallelCommandGroup(
                  new RunHopperIntake(m_hopperIntake, HOPPERINTAKE.INTAKE_SPEED.INTAKING),
                  new RunEndEffectorIntake(m_endEffector, ROLLER_SPEED.INTAKE_CORAL)
                      .until(m_endEffector::hasCoral),
                  moveSuperStructure(
                      ELEVATOR_SETPOINT.INTAKE_HOPPER, PIVOT_SETPOINT.INTAKE_HOPPER)))
          .onFalse(
              moveSuperStructure(ELEVATOR_SETPOINT.START_POSITION, PIVOT_SETPOINT.STOWED)
                  .withTimeout(1));
      // Coral Reverse / Algae Outtake
      m_driverController
          .povDown()
          .whileTrue(
              new ConditionalCommand(
                  new RunEndEffectorIntake(m_endEffector, ROLLER_SPEED.OUTTAKE_ALGAE_PROCESSOR),
                  new ParallelCommandGroup(
                      new RunHopperIntake(m_hopperIntake, HOPPERINTAKE.INTAKE_SPEED.FREEING_CORAL),
                      new RunEndEffectorIntake(m_endEffector, ROLLER_SPEED.CORAL_REVERSE),
                      moveSuperStructure(
                          ELEVATOR_SETPOINT.INTAKE_HOPPER, PIVOT_SETPOINT.INTAKE_HOPPER)),
                  () -> m_selectedGamePiece == ROBOT.GAME_PIECE.ALGAE))
          .onFalse(
              moveSuperStructure(ELEVATOR_SETPOINT.START_POSITION, PIVOT_SETPOINT.STOWED)
                  .withTimeout(1));
    }

    if (m_endEffector != null) {
      // Score Coral / Algae Intake
      m_driverController
          .rightTrigger()
          .whileTrue(
              new ConditionalCommand(
                  new RunEndEffectorIntake(m_endEffector, ROLLER_SPEED.INTAKE_ALGAE_REEF),
                  new RunEndEffectorIntake(m_endEffector, ROLLER_SPEED.OUTTAKE_CORAL),
                  () -> m_selectedGamePiece == ROBOT.GAME_PIECE.ALGAE));
      m_driverController
          .rightBumper()
          .whileTrue(
              new ConditionalCommand(
                  new RunEndEffectorIntake(m_endEffector, ROLLER_SPEED.OUTTAKE_ALGAE_PROCESSOR),
                  new RunEndEffectorIntake(m_endEffector, ROLLER_SPEED.CORAL_REVERSE),
                  () -> m_selectedGamePiece == ROBOT.GAME_PIECE.ALGAE));
    }

    if (m_climber != null) {
      m_driverController.back().whileTrue(new RunClimberVoltage(m_climber, Volts.of(2.5)));
      m_driverController
          .start()
          .whileTrue(
              Commands.startEnd(
                  () -> m_hopperIntake.moveServo(1.0),
                  () -> m_hopperIntake.stopServo())); // mvoe hopper out of the way
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

    // A bit messy, but it works
  }

  public void disabledPeriodic() {
    if (Controls.isRedAlliance()) {
      m_fieldSim.initializePoses("Red Branches", FIELD.RED_BRANCHES);
      m_fieldSim.initializePoses("Red Branch Targets", FIELD.RED_BRANCH_TARGETS);
      m_fieldSim.initializePoses("Blue Branches", new Pose2d(-5, -5, Rotation2d.kZero));
      m_fieldSim.initializePoses("Blue Branch Targets", new Pose2d(-5, -5, Rotation2d.kZero));
    } else {
      m_fieldSim.initializePoses("Red Branches", new Pose2d(-5, -5, Rotation2d.kZero));
      m_fieldSim.initializePoses("Red Branch Targets", new Pose2d(-5, -5, Rotation2d.kZero));
      m_fieldSim.initializePoses("Blue Branches", FIELD.BLUE_BRANCHES);
      m_fieldSim.initializePoses("Blue Branch Targets", FIELD.BLUE_BRANCH_TARGETS);
    }
  }

  public void autonomousInit() {
    m_swerveDrive.setNeutralMode(SWERVE.MOTOR_TYPE.ALL, NeutralModeValue.Brake);
  }

  public void teleopInit() {
    m_swerveDrive.setNeutralMode(SWERVE.MOTOR_TYPE.ALL, NeutralModeValue.Brake);
    m_elevator.teleopInit();
    m_hopperIntake.teleopInit();
    m_endEffectorPivot.teleopInit();
  }

  public void testInit() {
    try {
      if (ROBOT.robotID.equals(ROBOT.ROBOT_ID.ALPHABOT)) m_coralOuttake.testInit();
    } catch (Exception e) {
      DriverStation.reportWarning(
          "[RobotContainer] testInit() could not run coralOuttake.testInit()!", e.getStackTrace());
    }
  }

  public void testPeriodic() {
    try {
      if (ROBOT.robotID.equals(ROBOT.ROBOT_ID.ALPHABOT)) m_coralOuttake.testPeriodic();
    } catch (Exception e) {
      DriverStation.reportWarning(
          "[RobotContainer] testPeriodic() could not run coralOuttake.testPeriodic()!",
          e.getStackTrace());
    }
  }

  public void simulationInit() {
    // m_fieldSim.addStaticPoses("ReefBranches", FIELD.REEF_BRANCHES.getAllPose2d());
    m_fieldSim.initializePoses("AprilTags", FIELD.APRIL_TAG.getAllAprilTagPoses());
    m_fieldSim.initializePoses("Red Zones", FIELD.RED_ZONES);
    m_fieldSim.initializePoses("Blue Zones", FIELD.BLUE_ZONES);
  }

  boolean isInit = false;

  public void simulationPeriodic() {
    if (!isInit) {
      simulationInit();
      isInit = true;
    }
    m_robot2d.updateRobot2d();
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
              m_swerveDrive.setAngleToTarget(
                  m_swerveDrive
                      .getState()
                      .Pose
                      .getTranslation()
                      .minus(robotToBranch[1].getTranslation())
                      .getAngle()
                      .minus(Rotation2d.k180deg));
            });
  }

  public void robotPeriodic() {
    // m_questNav.periodic();

    // TODO: Implement code to drive to this Pose2d
    robotToBranch[0] = m_swerveDrive.getState().Pose;
    if (Controls.isBlueAlliance()) {
      nearestBranchPose = robotToBranch[0].nearest(Arrays.asList(FIELD.RED_BRANCHES));
    } else {
      nearestBranchPose = robotToBranch[0].nearest(Arrays.asList(FIELD.BLUE_BRANCHES));
    }
    robotToBranch[1] = FIELD.REEF_BRANCHES.getBranchPoseToTargetPose(nearestBranchPose);
    m_fieldSim.addPoses("LineToNearestBranch", robotToBranch);
  }
}
