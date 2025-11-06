// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.autos;

import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.util.FileVersionException;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import java.io.IOException;
import org.json.simple.parser.ParseException;
import org.team4201.codex.simulation.FieldSim;

public class PlotAutoPath extends Command {
  private final CommandSwerveDrivetrain m_swerveDrive;
  private final FieldSim m_fieldSim;
  private final PathPlannerPath m_path;

  public PlotAutoPath(CommandSwerveDrivetrain swerveDrive, FieldSim fieldSim, String pathName)
      throws FileVersionException, IOException, ParseException {
    this(swerveDrive, fieldSim, PathPlannerPath.fromPathFile(pathName));
  }

  public PlotAutoPath(
      CommandSwerveDrivetrain swerveDrive, FieldSim fieldSim, PathPlannerPath path) {
    m_swerveDrive = swerveDrive;
    m_fieldSim = fieldSim;
    m_path = path;

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(m_fieldSim);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    if (!DriverStation
        .isFMSAttached()) { // This causes significant delay on the autos. During competition we
      // don't want this
      try {
        var trajectory = m_swerveDrive.getTrajectoryUtils().getTrajectoryFromPathPlanner(m_path);
        m_fieldSim.addTrajectory(trajectory);
      } catch (Exception e) {
        System.out.println("Could not plot auto path ");
      }
    }
  }

  @Override
  public boolean runsWhenDisabled() {
    return true;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return true;
  }
}
