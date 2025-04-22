package frc.robot.commands;

import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.trajectory.PathPlannerTrajectory;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.units.measure.Time;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Drivetrain;

public class DriveOnPlannedPath extends Command {
  PathPlannerTrajectory trajectory;
  Drivetrain d;

  DriveOnPlannedPath(Drivetrain d, String path) {
    this.d = d;
    try {
      trajectory = PathPlannerPath.fromPathFile(path).generateTrajectory(null, null, null);
    } catch (Exception e) {
      e.printStackTrace();
    }
  }

  @Override
  public void initialize() {
  }

  @Override
  public void execute() {
    Pose2d pos = trajectory.sample(Timer.getMatchTime()).pose;
    d.driveRobot(pos.getX(), pos.getY(), pos.getRotation().getDegrees());
  }

  @Override
  public boolean isFinished() {
    return d.getPose() == trajectory.getEndState().pose;
  }
}
