package frc.robot.commands.swerve;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.CommandSwerveDrivetrain;

public class LockMovementBackwards extends Command {

  private final CommandSwerveDrivetrain m_swerveDrive;
  private double initialAngle = 0;

  
  public LockMovementBackwards(CommandSwerveDrivetrain swerveDrive) {
    m_swerveDrive = swerveDrive;
    addRequirements(m_swerveDrive);
  }

  @Override
  public void initialize() {
    initialAngle = m_swerveDrive.getPigeon2().getYaw().getValueAsDouble();
    m_swerveDrive.resetGyro(0);
  }

  @Override
  public void execute() {}

  @Override
  public void end(boolean interrupted) {
    m_swerveDrive.resetGyro(-initialAngle);
  }

  public boolean isFinished() {
    return false;
  }
}
