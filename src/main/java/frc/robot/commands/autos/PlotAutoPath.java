// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.autos;

import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.path.PathPlannerPath;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.Controls;
import java.util.ArrayList;

import org.team4201.codex.simulation.FieldSim;
import org.team4201.codex.utils.TrajectoryUtils;

public class PlotAutoPath extends Command {
  private final CommandSwerveDrivetrain m_swerveDrive;
  private final FieldSim m_fieldSim;
  private final ArrayList<PathPlannerPath> m_paths = new ArrayList<>();
  private ArrayList<Trajectory.State> m_pathPoints;
  private final String m_pathName;

  public PlotAutoPath(CommandSwerveDrivetrain swerveDrive, FieldSim fieldSim, String pathName) {
    m_swerveDrive = swerveDrive;
    m_fieldSim = fieldSim;
    m_pathName = pathName;

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(m_fieldSim);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    try {
      var path = PathPlannerPath.fromPathFile(m_pathName);
      
      var trajectory = m_swerveDrive.getTrajectoryUtils().getTrajectoryFromPathPlanner(path);
      m_fieldSim.addTrajectory(trajectory);
    } catch (Exception e) {
      System.out.println("Could not plot auto path " + m_pathName);
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
