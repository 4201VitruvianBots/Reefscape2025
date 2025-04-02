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
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.*;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.commands.RunHopperIntake;
import frc.robot.commands.ToggleGamePiece;
import frc.robot.commands.alphabot.RunAlgaeIntake;
import frc.robot.commands.alphabot.RunCoralOuttake;
import frc.robot.commands.autos.*;
import frc.robot.commands.climber.RunClimberVoltage;
import frc.robot.commands.climber.RunClimberVoltageJoystick;
import frc.robot.commands.climber.V2.RunV2ClimberVoltage;
import frc.robot.commands.elevator.SetElevatorSetpoint;
import frc.robot.commands.endEffector.EndEffectorBarge;
import frc.robot.commands.endEffector.EndEffectorJoystick;
import frc.robot.commands.endEffector.EndEffectorSetpoint;
import frc.robot.commands.endEffector.RunEndEffectorIntake;
import frc.robot.commands.swerve.DriveToTarget;
import frc.robot.commands.swerve.ResetGyro;
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
import frc.robot.generated.AlphaBotConstants;
import frc.robot.generated.V2Constants;
import frc.robot.generated.V3Constants;
import frc.robot.subsystems.*;
import frc.robot.subsystems.V2.V2Climber;
import frc.robot.subsystems.alphabot.AlgaeIntake;
import frc.robot.subsystems.alphabot.CoralOuttake;
import frc.robot.utils.Robot2d;
import frc.robot.utils.SysIdUtils;
import frc.robot.utils.Telemetry;
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

  @Logged(name = "Vision", importance = Logged.Importance.INFO)
  private final Vision m_vision = new Vision(m_controls);

  // AlphaBot subsystems
  private CoralOuttake m_coralOuttake;
  private AlgaeIntake m_algaeIntake;

  // V2/V3 subsystems
  //  @NotLogged(name = "V2Climber", importance = Logged.Importance.INFO)
  @NotLogged private V2Climber m_v2Climber;

  //  @Logged(name = "Climber", importance = Logged.Importance.INFO)
  @NotLogged private Climber m_climber;

  @Logged(name = "Elevator", importance = Logged.Importance.INFO)
  private Elevator m_elevator;

  @Logged(name = "EndEffector", importance = Logged.Importance.INFO)
  private EndEffector m_endEffector;

  @Logged(name = "EndEffectorPivot", importance = Logged.Importance.INFO)
  private EndEffectorPivot m_endEffectorPivot;

  @Logged(name = "HopperIntake", importance = Logged.Importance.INFO)
  private HopperIntake m_hopperIntake;

  @NotLogged private final Robot2d m_robot2d = new Robot2d();

  // Replace with CommandPS4Controller or CommandJoystick if needed
  @Logged(name = "AutoChooser")
  private final SendableChooser<Command> m_chooser = new SendableChooser<>();

  @NotLogged private final SendableChooser<Command> m_sysidChooser = new SendableChooser<>();

  @NotLogged
  private final CommandXboxController m_driverController =
      new CommandXboxController(USB.driver_xBoxController);

  @NotLogged
  private final CommandXboxController m_operatorController =
      new CommandXboxController(USB.operator_xBoxController);

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

  @NotLogged boolean isInit = false;

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    initializeSubSystems();
    initSmartDashboard();

    // Configure the trigger bindings
    if (ROBOT.robotID.equals(ROBOT.ROBOT_ID.ALPHABOT)) configureAlphaBotBindings();
    else configureBindings();

    // Only keep joystick warnings when FMS is attached
    if (!DriverStation.isFMSAttached()) {
      DriverStation.silenceJoystickConnectionWarning(true);
    }
  }

  private void initializeSubSystems() {
    // Initialize Subsystem classes
    if (ROBOT.robotID.equals(ROBOT.ROBOT_ID.V3)) {
      MaxSpeed =
          V3Constants.kSpeedAt12Volts.in(MetersPerSecond); // kSp,m,waeedAt12Volts desired top speed
      m_swerveDrive = V3Constants.createDrivetrain();
      m_elevator = new Elevator();
      m_endEffector = new EndEffector();
      m_endEffectorPivot = new EndEffectorPivot();
      //      m_v2Climber = new V2Climber();
      m_hopperIntake = new HopperIntake();
    } else if (ROBOT.robotID.equals(ROBOT.ROBOT_ID.V2)) {
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
    m_vision.registerFieldSim(m_fieldSim);

    m_telemetry.registerFieldSim(m_fieldSim);

    m_robot2d.registerSubsystem(m_elevator);
    m_robot2d.registerSubsystem(m_endEffectorPivot);
    m_robot2d.registerSubsystem(m_endEffector);

    // Set Subsystem DefaultCommands
    m_swerveDrive.setDefaultCommand(
        // Drivetrain will execute this command periodically
        m_swerveDrive.applyRequest(
            () -> {
              var rotationRate = m_driverController.getRightX() * MaxAngularRate;
              // // if heading target
              // if (m_swerveDrive.isTrackingState()) {
              //   rotationRate = m_swerveDrive.calculateRotationToTarget();
              // }
              drive
                  .withVelocityX(
                      m_driverController.getLeftY()
                          * MaxSpeed) // Drive forward with negative Y (forward)
                  .withVelocityY(
                      m_driverController.getLeftX() * MaxSpeed) // Drive left with negative X (left)
                  .withRotationalRate(
                      rotationRate); // Drive counterclockwise with negative X (left)
              return drive;
            }));
    if (m_elevator != null) {
      //   m_elevator.setDefaultCommand(
      //       new RunElevatorJoystick(
      //           m_elevator, () -> -m_driverController.getLeftY())); // Elevator open loop control
    }
    if (m_climber != null) {
      m_climber.setDefaultCommand(
          new RunClimberVoltageJoystick(m_climber, () -> -m_operatorController.getLeftY()));
    }
    if (m_endEffectorPivot != null) {
      // m_endEffectorPivot.setDefaultCommand(
      //     new EndEffectorJoystick(m_endEffectorPivot, () -> -m_driverController.getRightY()));
    }
  }

  private void initAutoChooser() {
    SmartDashboard.putData("Auto Mode", m_chooser);
    m_chooser.setDefaultOption("Do Nothing", new WaitCommand(0));

    m_chooser.addOption("DriveForward", new DriveForward(m_swerveDrive, m_fieldSim));
    // m_chooser.addOption("TestAuto1", new TestAuto1(m_swerveDrive, m_fieldSim));
    m_chooser.addOption(
        "OnePieceMiddle",
        new OnePiece(
            m_swerveDrive, m_fieldSim, m_elevator, m_endEffectorPivot, m_endEffector, m_vision));

    // m_chooser.addOption(
    //     "HopperTest",
    //     new TestHopperAuto(
    //         m_swerveDrive,
    //         m_fieldSim,
    //         m_elevator,
    //         m_endEffector,
    //         m_endEffectorPivot,
    //         m_hopperIntake));

    m_chooser.addOption(
        "ThreePieceRight",
        new ThreePieceRight(
            m_swerveDrive,
            m_fieldSim,
            m_elevator,
            m_endEffectorPivot,
            m_endEffector,
            m_hopperIntake));

    // m_chooser.addOption(
    //     "TwoPiece",
    //     new TwoPiece(
    //         m_swerveDrive,
    //         m_fieldSim,
    //         m_elevator,
    //         m_endEffectorPivot,
    //         m_endEffector,
    //         m_hopperIntake));

    // m_chooser.addOption(
    //     "OnePieceLeft",
    //     new OnePieceLeft(
    //         m_swerveDrive,
    //         m_fieldSim,
    //         m_elevator,
    //         m_endEffectorPivot,
    //         m_endEffector,
    //         m_hopperIntake));

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
        "ThreePieceLeft",
        new ThreePieceLeft(
            m_swerveDrive,
            m_fieldSim,
            m_elevator,
            m_endEffectorPivot,
            m_endEffector,
            m_hopperIntake));
    // m_chooser.addOption(
    //     "VisionTest",
    //     new VisionTest(
    //         m_swerveDrive,
    //         m_fieldSim,
    //         m_elevator,
    //         m_endEffectorPivot,
    //         m_endEffector,
    //         m_hopperIntake));
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
    var driveToTarget = new DriveToTarget(m_swerveDrive, m_vision);

    m_operatorController.leftBumper().whileTrue(driveToTarget.generateCommand(true));
    m_operatorController.rightBumper().whileTrue(driveToTarget.generateCommand(false));

    if (m_coralOuttake != null) {
      // TODO: Make speeds into enum setpoints
      m_operatorController
          .leftBumper()
          .whileTrue(new RunCoralOuttake(m_coralOuttake, 0.15)); // outtake
      m_operatorController
          .rightBumper()
          .whileTrue(new RunCoralOuttake(m_coralOuttake, -0.15)); // intake
    }

    if (m_algaeIntake != null) {
      // TODO: Make speeds into enum setpoints
      m_operatorController.x().whileTrue(new RunAlgaeIntake(m_algaeIntake, 0.5)); // outtake
      m_operatorController.y().whileTrue(new RunAlgaeIntake(m_algaeIntake, -0.5)); // intake
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

  private void configureBindings() {
    var driveToTarget = new DriveToTarget(m_swerveDrive, m_vision);

    m_operatorController.leftBumper().whileTrue(driveToTarget.generateCommand(true));
    m_operatorController.rightBumper().whileTrue(driveToTarget.generateCommand(false));

    // Algae Toggle
    m_operatorController.leftBumper().onTrue(new ToggleGamePiece(m_controls));

    if (m_elevator != null && m_endEffectorPivot != null) {
      m_operatorController
          .a()
          .whileTrue(
              new ConditionalCommand(
                  moveSuperStructure(
                      ELEVATOR_SETPOINT.ALGAE_REEF_INTAKE_LOWER,
                      PIVOT_SETPOINT.INTAKE_ALGAE_LOW), // Algae L2
                  moveSuperStructure(ELEVATOR_SETPOINT.LEVEL_2, PIVOT_SETPOINT.L3_L2), // Coral L2
                  m_controls::isGamePieceAlgae))
          .onFalse(
              new ConditionalCommand(
                  moveSuperStructure(
                          ELEVATOR_SETPOINT.PROCESSOR, PIVOT_SETPOINT.OUTTAKE_ALGAE_PROCESSOR)
                      .withTimeout(1),
                  moveSuperStructure(ELEVATOR_SETPOINT.START_POSITION, PIVOT_SETPOINT.STOWED)
                      .withTimeout(1),
                  m_controls::isGamePieceAlgae));

      m_operatorController
          .x()
          .whileTrue(
              new ConditionalCommand(
                  moveSuperStructure(
                      ELEVATOR_SETPOINT.PROCESSOR,
                      PIVOT_SETPOINT.OUTTAKE_ALGAE_PROCESSOR), // Algae L1
                  moveSuperStructure(
                      ELEVATOR_SETPOINT.START_POSITION, PIVOT_SETPOINT.STOWED), // Coral L1
                  m_controls::isGamePieceAlgae))
          .onFalse(
              new ConditionalCommand(
                  moveSuperStructure(
                          ELEVATOR_SETPOINT.PROCESSOR, PIVOT_SETPOINT.OUTTAKE_ALGAE_PROCESSOR)
                      .withTimeout(1),
                  moveSuperStructure(ELEVATOR_SETPOINT.START_POSITION, PIVOT_SETPOINT.STOWED)
                      .withTimeout(1),
                  m_controls::isGamePieceAlgae));

      m_operatorController
          .y()
          .whileTrue(
              new ConditionalCommand(
                  moveSuperStructureDelayed(
                      ELEVATOR_SETPOINT.LEVEL_4, PIVOT_SETPOINT.BARGE), // Algae L4
                  moveSuperStructureDelayed(
                      ELEVATOR_SETPOINT.LEVEL_4, PIVOT_SETPOINT.L4), // Coral L4
                  m_controls::isGamePieceAlgae))
          .onFalse(
              new ConditionalCommand(
                  new SequentialCommandGroup(
                          new EndEffectorSetpoint(
                                  m_endEffectorPivot, PIVOT_SETPOINT.OUTTAKE_ALGAE_PROCESSOR)
                              .withTimeout(0.7),
                          new SetElevatorSetpoint(m_elevator, ELEVATOR_SETPOINT.PROCESSOR))
                      .withTimeout(1),
                  new SequentialCommandGroup(
                          new EndEffectorSetpoint(m_endEffectorPivot, PIVOT_SETPOINT.STOWED)
                              .withTimeout(0.7),
                          new SetElevatorSetpoint(m_elevator, ELEVATOR_SETPOINT.START_POSITION))
                      .withTimeout(1),
                  m_controls::isGamePieceAlgae));

      m_operatorController
          .b()
          .whileTrue(
              new ConditionalCommand(
                  moveSuperStructure(
                      ELEVATOR_SETPOINT.ALGAE_REEF_INTAKE_UPPER,
                      PIVOT_SETPOINT.INTAKE_ALGAE_HIGH), // Algae L3
                  moveSuperStructure(ELEVATOR_SETPOINT.LEVEL_3, PIVOT_SETPOINT.L3_L2), // Coral L3
                  m_controls::isGamePieceAlgae))
          .onFalse(
              new ConditionalCommand(
                  moveSuperStructure(
                          ELEVATOR_SETPOINT.PROCESSOR, PIVOT_SETPOINT.OUTTAKE_ALGAE_PROCESSOR)
                      .withTimeout(1),
                  moveSuperStructure(ELEVATOR_SETPOINT.START_POSITION, PIVOT_SETPOINT.STOWED)
                      .withTimeout(1),
                  m_controls::isGamePieceAlgae));
    }

    // Ground intake on left trigger, TODO: implement
    // Ground intake algae on povDown, TODO: implement

    if (m_hopperIntake != null
        && m_endEffectorPivot != null
        && m_endEffector != null
        && m_elevator != null) {
      // Ready hopper
      m_operatorController
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
      m_operatorController
          .povDown()
          .whileTrue(
              new ConditionalCommand(
                  new ParallelCommandGroup(
                      new SequentialCommandGroup(
                          new RunEndEffectorIntake(
                                  m_endEffector, ROLLER_SPEED.HOLD_ALGAE_BARGE_FLICK)
                              .withTimeout(0.15),
                          new RunEndEffectorIntake(m_endEffector, ROLLER_SPEED.OUTTAKE_ALGAE_BARGE)
                              .withTimeout(0.5)),
                      new EndEffectorBarge(m_endEffectorPivot)), // Net scoring binding here
                  new ParallelCommandGroup(
                      new RunHopperIntake(m_hopperIntake, HOPPERINTAKE.INTAKE_SPEED.FREEING_CORAL),
                      new RunEndEffectorIntake(m_endEffector, ROLLER_SPEED.CORAL_REVERSE),
                      moveSuperStructure(
                          ELEVATOR_SETPOINT.INTAKE_HOPPER, PIVOT_SETPOINT.INTAKE_HOPPER)),
                  m_controls::isGamePieceAlgae))
          .onFalse(
              new ConditionalCommand(
                  new EndEffectorSetpoint(m_endEffectorPivot, PIVOT_SETPOINT.INTAKE_ALGAE_HIGH)
                      .withTimeout(1.0),
                  moveSuperStructure(ELEVATOR_SETPOINT.START_POSITION, PIVOT_SETPOINT.STOWED)
                      .withTimeout(1.0),
                  m_controls::isGamePieceAlgae));
    }

    if (m_endEffector != null) {
      // Score Coral / Algae Intake
      m_operatorController
          .rightTrigger()
          .whileTrue(
              new ConditionalCommand(
                  new RunEndEffectorIntake(m_endEffector, ROLLER_SPEED.INTAKE_ALGAE_REEF),
                  new RunEndEffectorIntake(m_endEffector, ROLLER_SPEED.OUTTAKE_CORAL),
                  m_controls::isGamePieceAlgae));

      m_operatorController
          .rightBumper()
          .whileTrue(
              new ConditionalCommand(
                  new RunEndEffectorIntake(m_endEffector, ROLLER_SPEED.OUTTAKE_ALGAE_PROCESSOR),
                  new RunEndEffectorIntake(m_endEffector, ROLLER_SPEED.CORAL_REVERSE),
                  m_controls::isGamePieceAlgae));
    }

    if (m_climber != null) {
      m_operatorController
          .back()
          .whileTrue(new RunClimberVoltage(m_climber, Volts.of(4.8))); // 40% output
    }
    if (m_v2Climber != null) {
      m_operatorController
          .back()
          .whileTrue(new RunV2ClimberVoltage(m_v2Climber, Volts.of(2.5))); // 20.8% output
    }

    if (m_hopperIntake != null) {
      m_operatorController
          .start()
          .whileTrue(
              Commands.startEnd(
                  () -> m_hopperIntake.moveServo(1.0),
                  () -> m_hopperIntake.stopServo())); // move hopper out of the way
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

    if (m_climber != null) {
      m_climber.disabledInit();
    }

    // A bit messy, but it works
  }

  public void disabledPeriodic() {
    if (RobotBase.isReal()) {
      if (Controls.isRedAlliance()) {
        // m_fieldSim.initializePoses("Red Coral Branches", FIELD.RED_CORAL_BRANCHES);
        // m_fieldSim.initializePoses("Red Coral Branches Targets", FIELD.RED_CORAL_TARGETS);
        // m_fieldSim.initializePoses("Red Algae Branches", FIELD.RED_ALGAE_BRANCHES);
        // m_fieldSim.initializePoses("Red Algae Branches Targets", FIELD.RED_ALGAE_TARGETS);
        m_fieldSim.initializePoses("Blue Coral Branches", new Pose2d(-5, -5, Rotation2d.kZero));
        m_fieldSim.initializePoses(
            "Blue Coral Branches Targets", new Pose2d(-5, -5, Rotation2d.kZero));
        m_fieldSim.initializePoses("Blue Algae Branches", new Pose2d(-5, -5, Rotation2d.kZero));
        m_fieldSim.initializePoses(
            "Blue Algae Branches Targets", new Pose2d(-5, -5, Rotation2d.kZero));
      } else {
        m_fieldSim.initializePoses("Red Coral Branches", new Pose2d(-5, -5, Rotation2d.kZero));
        m_fieldSim.initializePoses(
            "Red Coral Branches Targets", new Pose2d(-5, -5, Rotation2d.kZero));
        m_fieldSim.initializePoses("Red Algae Branches", new Pose2d(-5, -5, Rotation2d.kZero));
        m_fieldSim.initializePoses(
            "Red Algae Branches Targets", new Pose2d(-5, -5, Rotation2d.kZero));
        // m_fieldSim.initializePoses("Blue Coral Branches", FIELD.BLUE_CORAL_BRANCHES);
        // m_fieldSim.initializePoses("Blue Coral Branches Targets", FIELD.BLUE_CORAL_TARGETS);
        // m_fieldSim.initializePoses("Blue Algae Branches", FIELD.BLUE_ALGAE_BRANCHES);
        // m_fieldSim.initializePoses("Blue Algae Branches Targets", FIELD.BLUE_ALGAE_TARGETS);
      }
    } else {
      // m_fieldSim.initializePoses("Red Coral Branches", FIELD.RED_CORAL_BRANCHES);
      // m_fieldSim.initializePoses("Red Coral Branches Targets", FIELD.RED_CORAL_TARGETS);
      // m_fieldSim.initializePoses("Red Algae Branches", FIELD.RED_ALGAE_BRANCHES);
      // m_fieldSim.initializePoses("Red Algae Branches Targets", FIELD.RED_ALGAE_TARGETS);
      // m_fieldSim.initializePoses("Blue Coral Branches", FIELD.BLUE_CORAL_BRANCHES);
      // m_fieldSim.initializePoses("Blue Coral Branches Targets", FIELD.BLUE_CORAL_TARGETS);
      // m_fieldSim.initializePoses("Blue Algae Branches", FIELD.BLUE_ALGAE_BRANCHES);
      // m_fieldSim.initializePoses("Blue Algae Branches Targets", FIELD.BLUE_ALGAE_TARGETS);
    }

    if (m_climber != null) {
      m_climber.disabledPeriodic();
    }
  }

  public void autonomousInit() {
    m_swerveDrive.setNeutralMode(SWERVE.MOTOR_TYPE.ALL, NeutralModeValue.Brake);
  }

  public void teleopInit() {
    m_swerveDrive.setNeutralMode(SWERVE.MOTOR_TYPE.ALL, NeutralModeValue.Brake);
    if (m_elevator != null) m_elevator.teleopInit();
    if (m_hopperIntake != null) m_hopperIntake.teleopInit();
    if (m_endEffectorPivot != null) m_endEffectorPivot.teleopInit();
    if (m_climber != null) m_climber.teleopInit();
  }

  public void teleopPeriodic() {
    m_vision.setTargetLock(m_operatorController.y().getAsBoolean());
    if (m_vision.isOnTarget()) {
      m_operatorController.getHID().setRumble(RumbleType.kBothRumble, 0.2);
    } else {
      m_operatorController.getHID().setRumble(RumbleType.kBothRumble, 0);
    }
  }

  public void testInit() {
    try {
      if (ROBOT.robotID.equals(ROBOT.ROBOT_ID.ALPHABOT)) m_coralOuttake.testInit();
    } catch (Exception e) {
      DriverStation.reportWarning(
          "[RobotContainer] testInit() could not run coralOuttake.testInit()!", e.getStackTrace());
    }
    if (m_elevator != null) m_elevator.testInit();
    if (m_endEffectorPivot != null) m_endEffectorPivot.testInit();
  }

  public void testPeriodic() {
    try {
      if (ROBOT.robotID.equals(ROBOT.ROBOT_ID.ALPHABOT)) m_coralOuttake.testPeriodic();
    } catch (Exception e) {
      DriverStation.reportWarning(
          "[RobotContainer] testPeriodic() could not run coralOuttake.testPeriodic()!",
          e.getStackTrace());
    }
    if (m_elevator != null) m_elevator.testPeriodic();
    if (m_endEffectorPivot != null) m_endEffectorPivot.testPeriodic();
  }

  public void simulationInit() {
    try {
      // This crashes code if used in Robot::simulationInit() due to race condition
      // m_fieldSim.addStaticPoses("ReefBranches", FIELD.REEF_BRANCHES.getAllPose2d());
      m_fieldSim.initializePoses("AprilTags", FIELD.APRIL_TAG.getAllAprilTagPoses());
      m_fieldSim.initializePoses("Red Zones", FIELD.RED_ZONES);
      m_fieldSim.initializePoses("Blue Zones", FIELD.BLUE_ZONES);
      isInit = true;
    } catch (Exception e) {
      DriverStation.reportWarning("[RobotContainer] Could not run simulationInit()", false);
    }
  }

  public void simulationPeriodic() {
    if (!isInit) {
      simulationInit();
    }
    m_robot2d.updateRobot2d();
  }

  public Controls getControls() {
    return m_controls;
  }
}
